from socket import *
import time
import array


# 2000端口控制类
class RobotCtrl:

    def __init__(self, server_ip='192.168.1.2', server_port=2000, delay=0.1):

        # 创建socket
        self.__tcp_client_socket = socket(AF_INET, SOCK_STREAM)

        # 链接服务器
        self.__tcp_client_socket.connect((server_ip, server_port))

        self.send_data = ''
        self.delay = delay

    # 发送数据
    def data_send(self):
        """数据发送并等待动作执行完成,默认延时0.2s"""

        self.__tcp_client_socket.send(self.send_data.encode('gbk'))
        __recv_data = ''
        while b'Script finish\n' != __recv_data:
            __recv_data = self.__tcp_client_socket.recv(1024)
        time.sleep(self.delay)

    # 移动控制函数movej、movel、movec
    def movej(self, j1, j2, j3, j4, j5, j6, v, a, r=-1):
        """
            函数说明：该指令控制机械臂从当前状态，按照关节运动的方式移动到目标关节角状态。
            参数说明：
            j1, j2, j3, j4, j5, j6：对应 1-6 关节的目标关节角度，单位 deg
            v：关节角速度，单位（系统设定速度的百分比%），取值范围（0,100]
            a：关节加速度，单位（系统设定加速度的百分比%），取值范围（0,100]
            r：关节融合半径，默认值为 0/-1，表示无融合。当数值大于 0时表示与下一条运动融合。
            返回值：movej指令字符串
        """
        str_point = '[' + str(j1) + ',' + str(j2) + ',' + str(j3) + ',' + str(j4) + ',' + str(j5) + ',' + str(j6) + '],'
        self.send_data = r'movej(' + str_point + str(v) + ',' + str(a) + ',' + str(r) + ');'
        RobotCtrl.data_send(self)

    def movel(self, x, y, z, rx, ry, rz, v, a, r=-1):
        """
            x, y, z：单位mm
            rx, ry, rz：单位deg
            v：单位mm/s
            a：单位mm/(s^2)
            r：单位deg
        """
        str_point = '[' + str(x) + ',' + str(y) + ',' + str(z) + ',' + str(rx) + ',' + str(ry) + ',' + str(rz) + '],'
        self.send_data = r'movel(' + str_point + str(v) + ',' + str(a) + ',' + str(r) + ');'
        RobotCtrl.data_send(self)

    def movec(self, x1, y1, z1, rx1, ry1, rz1, x2, y2, z2, rx2, ry2, rz2, v, a, r=-1):
        """
            x, y, z：单位mm
            rx, ry, rz：单位deg
            v：单位mm/s
            a：单位mm/(s^2)
            r：单位deg
        """
        str_point1 = '[' + str(x1) + ',' + str(y1) + ',' + str(z1) + ',' + str(rx1) + ',' + str(ry1) + ',' + str(
            rz1) + '],'
        str_point2 = '[' + str(x2) + ',' + str(y2) + ',' + str(z2) + ',' + str(rx2) + ',' + str(ry2) + ',' + str(
            rz2) + '],'

        self.send_data = r'movec(' + str_point1 + str_point2 + str(v) + ',' + str(a) + ',' + str(r) + ');'
        RobotCtrl.data_send(self)

    def movej_pose(self, x, y, z, rx, ry, rz, v, a, r=-1):
        """
            x, y, z：单位mm
            rx, ry, rz：单位deg
            v：单位mm/s
            a：单位mm/(s^2)
            r：单位deg
        """
        str_point = '[' + str(x) + ',' + str(y) + ',' + str(z) + ',' + str(rx) + ',' + str(ry) + ',' + str(rz) + '],'

        self.send_data = r'movej_pose(' + str_point + str(v) + ',' + str(a) + ',' + str(r) + ');'
        RobotCtrl.data_send(self)

    def tcp_move(self, x, y, z, rx, ry, rz, v, a, r=-1):
        """
            x, y, z：单位mm
            rx, ry, rz：单位deg
            v：单位mm/s
            a：单位mm/(s^2)
            r：单位deg
        """
        str_point = '[' + str(x) + ',' + str(y) + ',' + str(z) + ',' + str(rx) + ',' + str(ry) + ',' + str(rz) + '],'

        self.send_data = r'tcp_move(' + str_point + str(v) + ',' + str(a) + ',' + str(r) + ');'
        RobotCtrl.data_send(self)

    def tcp_move_2p(self, x1, y1, z1, rx1, ry1, rz1, x2, y2, z2, rx2, ry2, rz2, v, a, r=-1):
        """
            x, y, z：单位mm
            rx, ry, rz：单位deg
            v：单位mm/s
            a：单位mm/(s^2)
            r：单位deg
        """
        str_point1 = '[' + str(x1) + ',' + str(y1) + ',' + str(z1) + ',' + str(rx1) + ',' + str(ry1) + ',' + str(
            rz1) + '],'
        str_point2 = '[' + str(x2) + ',' + str(y2) + ',' + str(z2) + ',' + str(rx2) + ',' + str(ry2) + ',' + str(
            rz2) + '],'

        self.send_data = r'tcp_move_2p(' + str_point1 + str_point2 + str(v) + ',' + str(a) + ',' + str(r) + ');'
        RobotCtrl.data_send(self)

    def spline(self, x, y, z, rx, ry, rz, v, a):
        """
            x, y, z：单位mm
            rx, ry, rz：单位deg
            v：单位mm/s
            a：单位mm/(s^2)
        """
        str_point = '[' + str(x) + ',' + str(y) + ',' + str(z) + ',' + str(rx) + ',' + str(ry) + ',' + str(rz) + '],'

        return  r'spline(' + str_point + str(v) + ',' + str(a) + ');'
        
    def spline_op(self, x, y, z, rx, ry, rz, v, a, t1=-1, n_io1=0, io1=0, t2=-1, n_io2=0, io2=0):
        """
            x, y, z：单位mm
            rx, ry, rz：单位deg
            v：单位mm/s
            a：单位mm/(s^2)
        """
        str_point = '[' + str(x) + ',' + str(y) + ',' + str(z) + ',' + str(rx) + ',' + str(ry) + ',' + str(rz) + '],'
        str_io = str(t1) + ',' + str(n_io1) + ',' + str(io1) + ',' + str(t2) + ',' + str(n_io2) + ',' + str(io2)

        return  r'spline(' + str_point + str(v) + ',' + str(a) + ',' + str_io + ');'

    def spline_start(self, *parameters):
        num = len(parameters)
        str_spline = ''
        if 0 < num < 50:
            for i in range(num):
                str_spline = str_spline + parameters[i]

            self.send_data = r'spline_start();' + str_spline
            RobotCtrl.data_send(self)

        else:
            print('节点最多49个，最少1个')

    def movel_op(self, x, y, z, rx, ry, rz, v, a, r=-1, t1=-1, n_io1=0, io1=0, t2=-1, n_io2=0, io2=0):
        """
            x, y, z：单位mm
            rx, ry, rz：单位deg
            v：单位mm/s
            a：单位mm/(s^2)
            r：单位deg
        """
        str_point = '[' + str(x) + ',' + str(y) + ',' + str(z) + ',' + str(rx) + ',' + str(ry) + ',' + str(rz) + '],'
        str_io = str(t1) + ',' + str(n_io1) + ',' + str(io1) + ',' + str(t2) + ',' + str(n_io2) + ',' + str(io2)

        self.send_data = r'movel_op(' + str_point + str(v) + ',' + str(a) + ',' + str(r) + ',' + str_io + ');'
        RobotCtrl.data_send(self)

    def movec_op(self, x1, y1, z1, rx1, ry1, rz1, x2, y2, z2, rx2, ry2, rz2, v, a, r=-1,
                 t1=-1, n_io1=0, io1=0, t2=-1, n_io2=0, io2=0):
        """
            x, y, z：单位mm
            rx, ry, rz：单位deg
            v：单位mm/s
            a：单位mm/(s^2)
            r：单位deg
        """
        str_point1 = '[' + str(x1) + ',' + str(y1) + ',' + str(z1) + ',' + str(rx1) + ',' + str(ry1) + ',' + str(
            rz1) + '],'
        str_point2 = '[' + str(x2) + ',' + str(y2) + ',' + str(z2) + ',' + str(rx2) + ',' + str(ry2) + ',' + str(
            rz2) + '],'
        str_io = str(t1) + ',' + str(n_io1) + ',' + str(io1) + ',' + str(t2) + ',' + str(n_io2) + ',' + str(io2)

        self.send_data = r'movec_op(' + str_point1 + str_point2 + str(v) + ',' + str(a) + ',' + str(r) + ',' \
                         + str_io + ');'
        RobotCtrl.data_send(self)

    def replay(self, n, speed):

        self.send_data = r'replay(' + str(n) + ',' + str(speed) + ');'
        RobotCtrl.data_send(self)

    # 坐标系
    def coordinate_tool(self, x, y, z, rx, ry, rz, mass=0, loadBiasX=0, loadBiasY=0, loadBiasZ=0):
        """
            x, y, z：单位mm
            rx, ry, rz：单位deg
            mass：质量，单位kg，默认0kg
            loadBiasX, loadBiasY, loadBiasZ：质心位置，单位mm，默认0mm
        """
        str_point = str(x) + ',' + str(y) + ',' + str(z) + ',' + str(rx) + ',' + str(ry) + ',' + str(rz)
        str_mass = str(mass) + ',' + str(loadBiasX) + ',' + str(loadBiasY) + ',' + str(loadBiasZ)
        self.send_data = r'coordinate_tool(' + str_point + ',' + str_mass + ');'
        RobotCtrl.data_send(self)

    def coordinate_user(self, x, y, z, rx, ry, rz):
        """
            x, y, z：单位mm
            rx, ry, rz：单位deg
        """
        str_point = str(x) + ',' + str(y) + ',' + str(z) + ',' + str(rx) + ',' + str(ry) + ',' + str(rz)
        self.send_data = r'coordinate_user(' + str_point + ');'
        RobotCtrl.data_send(self)

    def coordinate_clear(self):

        self.send_data = r'coordinate_clear();'
        RobotCtrl.data_send(self)

    def set_tcp_load(self, tcpLoad, loadBiasX, loadBiasY, loadBiasZ):
        """
        tcpLoad: 负载质量（单位：kg）
        loadBiasX, loadBiasY, loadBiasZ: 负载质心相对末端偏移（单位：mm），
        使用函数时这 3 个参数可省略，省略后系统会使用当前设置的负载质心偏移进行计算。
        """
        str_load = str(tcpLoad) + ',' + str(loadBiasX) + ',' + str(loadBiasY) + ',' + str(loadBiasZ)

        self.send_data = r'set_tcp_load(' + str_load + ');'
        RobotCtrl.data_send(self)

    # 工具IO输出
    def io_out(self, num, value):

        self.send_data = r'io_out(' + str(num) + ',' + str(value) + ');'
        RobotCtrl.data_send(self)

    def tool_io_out(self, num, value):

        self.send_data = r'tool_io_out(' + str(num) + ',' + str(value) + ');'
        RobotCtrl.data_send(self)

    def io_v_out(self, num, value):

        self.send_data = r'io_v_out(' + str(num) + ',' + str(value) + ');'
        RobotCtrl.data_send(self)

    def set_do_reactive(self, num, value):

        self.send_data = r'set_do_reactive(' + str(num) + ',' + str(value) + ');'
        RobotCtrl.data_send(self)

    # 调用机械臂内置程序
    def run(self, name, speed=-1):

        if speed == -1:
            send_data = r'run(' + name + '.spf)'
            self.__tcp_client_socket.send(send_data.encode('gbk'))
        elif 0 < speed <= 100:
            send_data = r'run(' + name + '.spf,' + str(speed) + ')'
            self.__tcp_client_socket.send(send_data.encode('gbk'))
        else:
            print(r'speed error')

    # 机械臂速度设置
    def speed_set(self, speed):
        send_data = r'speed(' + str(speed) + ')'
        self.__tcp_client_socket.send(send_data.encode('gbk'))
        return self.__tcp_client_socket.recv(1024).decode('utf-8')

    # 机械臂停止当前指令
    def stop(self):
        self.__tcp_client_socket.send(r'stop'.encode('gbk'))
        return self.__tcp_client_socket.recv(1024).decode('utf-8')

    # 机械臂暂停当前指令
    def pause(self):
        self.__tcp_client_socket.send(r'pause'.encode('gbk'))
        return self.__tcp_client_socket.recv(1024).decode('utf-8')

    # 机械臂在暂停时会继续运行程序
    def resume(self):
        self.__tcp_client_socket.send(r'resume'.encode('gbk'))
        return self.__tcp_client_socket.recv(1024).decode('utf-8')

    # 错误复位
    def reset(self):
        self.__tcp_client_socket.send(r'reset'.encode('gbk'))
        return self.__tcp_client_socket.recv(1024).decode('utf-8')

    # 登录
    def login(self):
        self.__tcp_client_socket.send(r'login'.encode('gbk'))
        return self.__tcp_client_socket.recv(1024).decode('utf-8')

    # 机械臂断开连接
    def disconnect(self):
        self.__tcp_client_socket.send(r'disconnect'.encode('gbk'))
        return self.__tcp_client_socket.recv(1024).decode('utf-8')

    # 机械臂关机
    def shutdown(self):
        self.__tcp_client_socket.send(r'shutdown'.encode('gbk'))
        return self.__tcp_client_socket.recv(1024).decode('utf-8')

    # 机械臂断电
    def poweroff(self):
        self.__tcp_client_socket.send(r'poweroff'.encode('gbk'))
        return self.__tcp_client_socket.recv(1024).decode('utf-8')

    # 读取机械臂末端位置信息
    def status(self):
        """返回当前机械臂位置信息"""
        send_data = r'status'
        self.__tcp_client_socket.send(send_data.encode('gbk'))

        recv_data = self.__tcp_client_socket.recv(1024).decode('utf-8')

        recv_data_new = recv_data.split(";")
        x = float(recv_data_new[0])
        y = float(recv_data_new[1])
        z = float(recv_data_new[2])
        rx = float(recv_data_new[3])
        ry = float(recv_data_new[4])
        rz = float(recv_data_new[5])
        j1 = float(recv_data_new[6])
        j2 = float(recv_data_new[7])
        j3 = float(recv_data_new[8])
        j4 = float(recv_data_new[9])
        j5 = float(recv_data_new[10])
        j6 = float(recv_data_new[11])
        # print(x, y, z, rx, ry, rz)

        return x, y, z, rx, ry, rz, j1, j2, j3, j4, j5, j6

    # 关节角度换算笛卡尔坐标
    def ikine(self, x, y, z, rx, ry, rz):
        send_data = r'ikine(' + str(x) + ',' + str(y) + ',' + str(z) + ',' + str(rx) + ',' + str(ry) + ',' + str(
            rz) + ')'
        self.__tcp_client_socket.send(send_data.encode('gbk'))
        return self.__tcp_client_socket.recv(1024).decode('utf-8')

    # 笛卡尔坐标换算关节角度
    def fkine(self, j1, j2, j3, j4, j5, j6):
        send_data = r'fkine(' + str(j1) + ',' + str(j2) + ',' + str(j3) + ',' + str(j4) + ',' + str(j5) + ',' + str(
            j6) + ')'
        self.__tcp_client_socket.send(send_data.encode('gbk'))
        return self.__tcp_client_socket.recv(1024).decode('utf-8')

    # 仿真/实机切换
    def simulate(self, flag):
        send_data = r'simulate(' + str(flag) + ')'
        self.__tcp_client_socket.send(send_data.encode('gbk'))
        return self.__tcp_client_socket.recv(1024).decode('utf-8')

    def close(self):
        self.__tcp_client_socket.close()


# 2001端口监听类
class RobotState:

    def __init__(self):
        self.jointActualPos = []
        self.jointSetPos = []
        self.jointVel = []
        self.jointAcc = []
        self.jointCurrent = []
        self.jointTemp = []
        self.driverTemp = []
        self.cartActualPos = []
        self.cartSetPos = []
        self.cartVel = []
        self.cartAcc = []
        self.tcpExternalForce = []
        self.analogInput = 0
        self.analogOutput = 0
        self.IKineJointPose = []
        self.FKineEndPose = []
        self.digitalInputs = []
        self.digitalOutputs = []
        self.toolIoIn = []
        self.toolIoOut = []
        self.toolButton = []
        self.collision = 0
        self.collisionAxis = 0
        self.emcStop = 0
        self.slaveReady = []
        self.robotState = 0
        self.virtualIoIn = []
        self.virtualIoOut = []
        self.collisionDetection = 0
        self.simulationExecution = 0

    def receive_data(self, server_ip='192.168.1.2'):

        # 创建socket
        __tcp_listen_socket = socket(AF_INET, SOCK_STREAM)
        __tcp_listen_socket.connect((server_ip, 2001))

        # 监听socket
        __receiveData = __tcp_listen_socket.recv(1024)

        # 关节实际位置jointActualPos
        __jointActualPos = []
        for __i in range(6):
            __jointActualPos.append(round(
                array.array('d', __receiveData[__i * 8: (__i + 1) * 8])[0], 3))
        self.jointActualPos = __jointActualPos

        # 关节设定位置jointSetPos
        __jointSetPos = []
        for __i in range(6):
            __jointSetPos.append(round(array.array('d', __receiveData[56 + __i * 8: 56 + (__i + 1) * 8])[0], 3))
        self.jointSetPos = __jointSetPos

        # 关节速度jointVel
        __jointVel = []
        for __i in range(6):
            __jointVel.append(round(array.array('d', __receiveData[112 + __i * 8: 112 + (__i + 1) * 8])[0], 3))
        self.jointVel = __jointVel

        # 关节加速度jointAcc
        __jointAcc = []
        for __i in range(6):
            __jointAcc.append(round(array.array('d', __receiveData[168 + __i * 8: 168 + (__i + 1) * 8])[0], 3))
        self.jointAcc = __jointAcc

        # 关节电流jointCurrent
        __jointCurrent = []
        for __i in range(6):
            __jointCurrent.append(round(array.array('d', __receiveData[224 + __i * 8: 224 + (__i + 1) * 8])[0], 3))
        self.jointCurrent = __jointCurrent

        # 关节温度jointTemp
        __jointTemp = []
        for __i in range(6):
            __jointTemp.append(round(array.array('d', __receiveData[280 + __i * 8: 280 + (__i + 1) * 8])[0], 3))
        self.jointTemp = __jointTemp

        # 驱动器温度driverTemp
        __driverTemp = []
        for __i in range(6):
            __driverTemp.append(round(array.array('d', __receiveData[336 + __i * 8: 336 + (__i + 1) * 8])[0], 3))
        self.driverTemp = __driverTemp

        # 笛卡尔实际位置cartActualPos
        __cartActualPos = []
        for __i in range(6):
            __cartActualPos.append(round(array.array('d', __receiveData[392 + __i * 8:392 + (__i + 1) * 8])[0], 3))
        self.cartActualPos = __cartActualPos

        # 笛卡尔设定位置cartSetPos
        __cartSetPos = []
        for __i in range(6):
            __cartSetPos.append(round(array.array('d', __receiveData[440 + __i * 8: 440 + (__i + 1) * 8])[0], 3))
        self.cartSetPos = __cartSetPos

        # 笛卡尔速度cartVel
        __cartVel = []
        for __i in range(6):
            __cartVel.append(round(array.array('d', __receiveData[488 + __i * 8: 488 + (__i + 1) * 8])[0], 3))
        self.cartVel = __cartVel

        # 笛卡尔加速度cartAcc
        __cartAcc = []
        for __i in range(6):
            __cartAcc.append(round(array.array('d', __receiveData[536 + __i * 8: 536 + (__i + 1) * 8])[0], 3))
        self.cartAcc = __cartAcc

        # 工具外力tcpExternalForce
        __tcpExternalForce = []
        for __i in range(6):
            __tcpExternalForce.append(round(
                array.array('d', __receiveData[584 + __i * 8:584 + (__i + 1) * 8])[0], 3))
        self.tcpExternalForce = __tcpExternalForce

        # 模拟输入analogInput
        __analogInput = round(array.array('d', __receiveData[632:640])[0], 3)
        self.analogInput = __analogInput

        # 模拟输出analogOutput
        __analogOutput = round(array.array('d', __receiveData[640:648])[0], 3)
        self.analogOutput = __analogOutput

        # 逆运动学关节位姿IKineJointPose
        __IKineJointPose = []
        for __i in range(6):
            __IKineJointPose.append(round(
                array.array('d', __receiveData[648 + __i * 8:648 + (__i + 1) * 8])[0], 3))
        self.IKineJointPose = __IKineJointPose

        # 正运动学末端位姿FKineEndPose
        __FKineEndPose = []
        for __i in range(6):
            __FKineEndPose.append(round(array.array('d', __receiveData[704 + __i * 8: 704 + (__i + 1) * 8])[0], 3))
        self.FKineEndPose = __FKineEndPose

        # 数字输入digitalInputs
        __digitalInputs = []
        for __i in range(32):
            __digitalInputs.append(round(array.array('B', __receiveData[752 + __i:752 + __i + 1])[0], 3))
        self.digitalInputs = __digitalInputs

        # 数字输出digitalOutputs
        __digitalOutputs = []
        for __i in range(32):
            __digitalOutputs.append(round(array.array('B', __receiveData[784 + __i:784 + __i + 1])[0], 3))
        self.digitalOutputs = __digitalOutputs

        # 工具IO输入toolIoIn
        __toolIoIn = []
        for __i in range(8):
            __toolIoIn.append(round(array.array('B', __receiveData[816 + __i:816 + __i + 1])[0], 3))
        self.toolIoIn = __toolIoIn

        # 工具IO输出toolIoOut
        __toolIoOut = []
        for __i in range(8):
            __toolIoOut.append(round(array.array('B', __receiveData[824 + __i:824 + __i + 1])[0], 3))
        self.toolIoOut = __toolIoOut

        # 工具按键状态toolButton
        __toolButton = []
        for __i in range(4):
            __toolButton.append(round(array.array('B', __receiveData[832 + __i:832 + __i + 1])[0], 3))
        self.toolButton = __toolButton

        # 碰撞collision
        __collision = round(array.array('B', __receiveData[836:837])[0], 3)
        self.collision = __collision

        # 碰撞轴collisionAxis
        __collisionAxis = round(array.array('B', __receiveData[837:838])[0], 3)
        self.collisionAxis = __collisionAxis

        # 急停emcStop
        __emcStop = round(array.array('B', __receiveData[838:839])[0], 3)
        self.emcStop = __emcStop

        # 从站就绪slaveReady
        __slaveReady = []
        for __i in range(6):
            __slaveReady.append(round(array.array('B', __receiveData[839 + __i:839 + __i + 1])[0], 3))
        self.slaveReady = __slaveReady

        # 机器人状态robotState
        __robotState = round(array.array('B', __receiveData[846:847])[0], 3)
        self.robotState = __robotState

        # 虚拟输入位寄存器virtualIoIn
        __virtualIoIn = []
        for __i in range(50):
            __virtualIoIn.append(round(array.array('B', __receiveData[847 + __i:847 + __i + 1])[0], 3))
        self.virtualIoIn = __virtualIoIn

        # 虚拟输出位寄存器virtualIoOut
        __virtualIoOut = []
        for __i in range(50):
            __virtualIoOut.append(round(array.array('B', __receiveData[897 + __i:897 + __i + 1])[0], 3))
        self.virtualIoOut = __virtualIoOut

        # 碰撞检测开关标志位collisionDetection
        __collisionDetection = round(array.array('B', __receiveData[947:948])[0], 3)
        self.collisionDetection = __collisionDetection

        # 仿真运行标志位simulationExecution
        __simulationExecution = round(array.array('B', __receiveData[948:949])[0], 3)
        self.simulationExecution = __simulationExecution

        # 关闭socket
        __tcp_listen_socket.close()


if __name__ == '__main__':
    # 2000端口使用方法：用于给机械臂发送控制命令
    # 链接2000端口
    robot_ctrl = RobotCtrl(server_ip='192.168.1.2')

    # 调用类方法，插入所需要执行的程序
    status_data = robot_ctrl.status()  # 例如读取当前机械臂位置信息
    print(status_data)

    # 关闭2000端口
    robot_ctrl.close()

    # 2001端口使用方法：用于读取机械臂参数
    # 创建对象，只需创建一次
    robot_state = RobotState()

    # 更新端口数据，每次读取时需要调用此方法
    robot_state.receive_data(server_ip='192.168.1.2')

    # 使用所需属性数据，插入所需要执行的程序
    print(robot_state.cartActualPos)  # 例如使用当前笛卡尔实际位置