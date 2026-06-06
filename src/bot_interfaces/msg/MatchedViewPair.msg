# Contains the set of sparse 2D-2D corresondences
# between two views
uint32 cam_id_a
uint32 cam_id_b
builtin_interfaces/Time timestamp_a
builtin_interfaces/Time timestamp_b
bot_interfaces/Point2d[] points_a
bot_interfaces/Point2d[] points_b
float32[] match_scores
