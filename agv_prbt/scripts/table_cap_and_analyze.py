from take_picture import take_picutre, undistort_pic
from get_marker_pose import get_blue_marker_pose


def table_cap_and_analyze(table_pic_file_path, table_calibrated_pic_file_path):
    """
    拍照并获取上料台相对位置
    """
    take_picutre(table_pic_file_path)
    undistort_pic(table_pic_file_path, table_calibrated_pic_file_path)
    table_x, table_y, table_angle = get_blue_marker_pose(table_calibrated_pic_file_path)
    return table_x, table_y, table_angle