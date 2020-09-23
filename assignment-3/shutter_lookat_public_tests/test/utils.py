import os
import subprocess
import rospy

def inspect_rostopic_info(node_name):
    """
    Helper function to check a node's connections
    :return: True if the node subscribes to the target_topic
    """
    out = subprocess.Popen(['rosnode', 'info', node_name],
                           stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT)
    stdout, stderr = out.communicate()

    if stderr is not None:
        print("Failed to run rosnode info. Error:\n{}".format(stderr.decode('utf-8')))
        return False

    stdout = stdout.decode('utf-8')
    headers = ["Publications:", "Subscriptions:", "Services:", "Connections:"]

    in_sub = False
    for line in stdout.split('\n'):
        line = line.strip()
        # rospy.logwarn(line)  # print output of rosnode info
        if line in headers:
            in_sub = False
            if line == "Subscriptions:":
                in_sub = True

        if in_sub and line == "* /target [shutter_lookat/Target]":
            return True

    return False

def compute_import_path(*args):
    test_dir = os.path.dirname(os.path.abspath(__file__))
    import_dir = os.path.abspath(os.path.join(test_dir, '..', '..'))
    for path in args:
        import_dir = os.path.abspath(os.path.join(import_dir, path))
    return import_dir
