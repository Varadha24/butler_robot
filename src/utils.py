import rospy

def log_info(message):
    rospy.loginfo(message)

def load_configuration(param_name, default_value):
    return rospy.get_param(param_name, default_value)
