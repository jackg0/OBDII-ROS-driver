import obd
import time
import rospy
import sys
from std_msgs.msg import Float64

rospy.init_node('obd2_ros_driver')

pubrpm = rospy.Publisher('obd/rpm', Float64, queue_size=10)
pubspeed = rospy.Publisher('obd/speed', Float64, queue_size=10)
pubthrottle = rospy.Publisher('obd/throttle', Float64, queue_size=10)
pubrelthrot = rospy.Publisher('obd/rel_throttle', Float64, queue_size=10)
pubaccd = rospy.Publisher('obd/acc_pedal_d', Float64, queue_size=10)
pubacce = rospy.Publisher('obd/acc_pedal_e', Float64, queue_size=10)
pubthrotact = rospy.Publisher('obd/throttle_act', Float64, queue_size=10)
pubengine = rospy.Publisher('obd/engine_load', Float64, queue_size=10)
pubpressure = rospy.Publisher('obd/pressure', Float64, queue_size=10)

def new_rpm(v):
    value = float(str(v).split(':')[-1].split(' ')[0])
    pubrpm.publish(value)
def new_speed(v):
    value = float(str(v).split(':')[-1].split(' ')[0])
    pubspeed.publish(value)
def new_relthrot(v):
    value = float(str(v).split(':')[-1].split(' ')[0])
    pubrelthrot.publish(value)
def new_throt(v):
    value = float(str(v).split(':')[-1].split(' ')[0])
    pubthrottle.publish(value)
def new_accd(v):
    value = float(str(v).split(':')[-1].split(' ')[0])
    pubaccd.publish(value)
def new_acce(v):
    value = float(str(v).split(':')[-1].split(' ')[0])
    pubacce.publish(value)
def new_throtact(v):
    value = float(str(v).split(':')[-1].split(' ')[0])
    pubthrotact.publish(value)
def new_engine(v):
    value = float(str(v).split(':')[-1].split(' ')[0])
    pubengine.publish(value)
def new_pressure(v):
    value = float(str(v).split(':')[-1].split(' ')[0])
    pubpressure.publish(value)
def new_fuel(v):
    value = float(str(v).split(':')[-1].split(' ')[0])
    pubfuel.publish(value)

def find_connection(ports):
    for port in ports:
        connection = obd.Async(port)
        if connection.status() == obd.OBDStatus.CAR_CONNECTED:
            return connection
    return None

if __name__ == '__main__':
    try:
        obd.logger.setLevel(obd.logging.WARNING)

        if len(sys.argv) < 2:
            ports = obd.scan_serial()
        else:
            ports = [sys.argv[1]]

        connection = find_connection(ports)

        if connection is not None:
            rospy.loginfo("OBD - Connection found for OBD {}...".format(connection.port_name()))

            connection.watch(obd.commands.RPM, callback=new_rpm)
            connection.watch(obd.commands.SPEED, callback=new_speed)
            connection.watch(obd.commands.THROTTLE_POS, callback=new_throt)
            connection.watch(obd.commands.RELATIVE_THROTTLE_POS, callback=new_relthrot)
            connection.watch(obd.commands.ACCELERATOR_POS_D, callback=new_accd)
            connection.watch(obd.commands.ACCELERATOR_POS_E, callback=new_acce)
            connection.watch(obd.commands.THROTTLE_ACTUATOR, callback=new_throtact)
            connection.watch(obd.commands.ENGINE_LOAD, callback=new_engine)
            connection.watch(obd.commands.BAROMETRIC_PRESSURE, callback=new_pressure)

            rospy.loginfo("OBD - Starting OBD connection...")
            connection.start()
            rospy.loginfo("OBD - OBD connected...")

            rospy.spin()

            connection.stop()
            rospy.loginfo("OBD - OBD disconnected...")

    except rospy.ROSInterruptException:
        rospy.loginfo("OBD - ROS interrupted.")
        pass
