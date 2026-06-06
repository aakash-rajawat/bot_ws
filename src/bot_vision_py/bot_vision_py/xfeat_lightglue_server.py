import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from bot_interfaces.srv import MatchedCorrespondences2d
from bot_interfaces.msg import Point2d, MatchedViewPair
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from modules.xfeat import XFeat
import torch


class XFeatLightGlueServer(Node):
    def __init__(self):
        super().__init__("xfeat_lightglue_server")
        
        self.declare_parameter(
            "image_topics",
            ["/left_camera/image_raw", "/right_camera/image_raw"],
        )

        self.img_topics_ = self.get_parameter("image_topics").value

        self.img_subs_ = []
        self.img_buffer_ = {}
        self.cv_bridge_ = CvBridge()
        self.buffer_len_ = 5
        self.image_callback_group_ = MutuallyExclusiveCallbackGroup()
        self.service_callback_group_ = MutuallyExclusiveCallbackGroup()

        for idx, topic in enumerate(self.img_topics_):
            self.img_buffer_[idx] = []
            self.img_subs_.append(
                self.create_subscription(
                    Image,
                    topic,
                    lambda msg, cam_idx=idx: self.image_sub_callback(cam_idx, msg),
                    10,
                    callback_group=self.image_callback_group_
                )
            )

        self.xfeat_ = XFeat()

        
        self.srv_ = self.create_service(
            MatchedCorrespondences2d,
            "/xfeat_lightglue",
            self.request_callback,
            callback_group=self.service_callback_group_,
        )

        self.get_logger().info(
            f"xfeat_lightglue_server initialized with image topics: {self.img_topics_}"
        )
            
    def image_sub_callback(self, cam_idx, msg: Image):
        self.img_buffer_[cam_idx].append(msg)
        if len(self.img_buffer_[cam_idx]) > self.buffer_len_:
            self.img_buffer_[cam_idx].pop(0)

    def request_callback(self, request: MatchedCorrespondences2d.Request, response: MatchedCorrespondences2d.Response):

        response.success = False
        response.message = ""
        response.ref_cam_id = 0
        response.matched_pairs = []

        self.get_logger().info(
            "request_callback entered: selected_cam_ids=%s selected_stamps=%d choose_ref=%s ref_cam_id=%d"
            % (
                list(request.selected_cam_ids),
                len(request.selected_stamps),
                str(request.choose_ref),
                request.ref_cam_id,
            )
        )

        if len(request.selected_cam_ids) != len(request.selected_stamps):
            response.message = "selected_cam_ids and selected_stamps must have the same length"
            return response

        if len(request.selected_cam_ids) < 2:
            response.message = "Received less than two images"
            return response
        

        selected_images = {}
        for cam_id, stamp in zip(request.selected_cam_ids, request.selected_stamps):
            if cam_id not in self.img_buffer_:
                response.message = f"Unknown camera id: {cam_id}"
                return response

            matched_msg = None
            for msg in self.img_buffer_[cam_id]:
                if (
                    msg.header.stamp.sec == stamp.sec
                    and msg.header.stamp.nanosec == stamp.nanosec
                ):
                    matched_msg = msg
                    break

            if matched_msg is None:
                response.message = f"Failed to find a message in image buffer for cam id: {cam_id} at stamp {stamp.sec}.{stamp.nanosec}"
                return response

            selected_images[cam_id] = matched_msg

        # Feature Dtection and Desciptors using XFeat 
        ordered_msgs = [selected_images[cam_id] for cam_id in request.selected_cam_ids]
        cv_images = [self.cv_bridge_.imgmsg_to_cv2(msg, desired_encoding="rgb8") for msg in ordered_msgs]
        img_tensors = [torch.from_numpy(img.copy()).permute(2, 0, 1).float() / 255.0 for img in cv_images]
        batch = torch.stack(img_tensors, dim=0)

        outputs = self.xfeat_.detectAndCompute(batch)
        for out, img in zip(outputs, cv_images):
            h, w = img.shape[:2]
            out["image_size"] = (w, h)

        # 2D-2D point correspondences using LightGlue
        keypoint_counts = [out["keypoints"].shape[0] for out in outputs]
        auto_ref_local_idx = max(range(len(outputs)), key=lambda i: keypoint_counts[i])
        auto_ref_cam_id = request.selected_cam_ids[auto_ref_local_idx]

        if request.choose_ref:
            response.ref_cam_id = auto_ref_cam_id
        else:
            response.ref_cam_id = request.ref_cam_id

        if response.ref_cam_id not in request.selected_cam_ids:
            response.message = f"Requested ref_cam_id {response.ref_cam_id} is not in selected_cam_ids"
            return response

        ref_local_idx = request.selected_cam_ids.index(response.ref_cam_id)
        ref_cam_id = request.selected_cam_ids[ref_local_idx]
        ref_feats = outputs[ref_local_idx]

        for other_local_idx, other_feats in enumerate(outputs):
            if other_local_idx == ref_local_idx:
                continue

            other_cam_id = request.selected_cam_ids[other_local_idx]

            mkpts_ref, mkpts_other, match_indices = self.xfeat_.match_lighterglue(
                ref_feats,
                other_feats,
            )

            matched_pair = MatchedViewPair()
            matched_pair.cam_id_a = ref_cam_id
            matched_pair.cam_id_b = other_cam_id
            matched_pair.timestamp_a = selected_images[ref_cam_id].header.stamp
            matched_pair.timestamp_b = selected_images[other_cam_id].header.stamp
            matched_pair.points_a = [
                Point2d(x=float(p[0]), y=float(p[1])) for p in mkpts_ref
            ]
            matched_pair.points_b = [
                Point2d(x=float(p[0]), y=float(p[1])) for p in mkpts_other
            ]
            matched_pair.match_scores = []
            response.matched_pairs.append(matched_pair)

        response.ref_stamp = selected_images[ref_cam_id].header.stamp
        response.success = True
        response.message = (
            f"Retrieved {len(selected_images)} images from the buffer; "
            f"built {len(response.matched_pairs)} matched view pairs"
        )
        self.get_logger().info(response.message)
        

        return response


def main(args=None):
    rclpy.init(args=args)
    node = XFeatLightGlueServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
