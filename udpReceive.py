import socket
import struct
from contextlib import closing

import matplotlib.pyplot as plt
import math
import serial
import sys
import struct
import time
import threading

class Capture:
    def __init__(self, dataSize = 460, isInvert = True):
        self.theta = [0] * dataSize
        self.distance = [0] * dataSize
        self.intensity = [0] * dataSize
        self.writePos = 0
        self.dataSize = dataSize
        self.thread = threading.Thread(target = self.getData)
        self.lock = threading.Lock()
        self.isInvert = isInvert
        self.dataObtained = False
        self.rpm = 0
        self.UDP_IP="192.168.0.107"
        self.UDP_PORT=9000


    def getDataUnit(self):
        sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        sock.bind((self.UDP_IP,self.UDP_PORT))

        with closing(sock):
            data,addr = sock.recvfrom(1024)
            print(len(data))
            while len(data) < 31 :
                data,addr = sock.recvfrom(1024)
                print(len(data))
            tmp = data[0:3+1] #データ受信
            # "<" : リトルエンディアン, "H" : 2バイト符号なしデータ, "B" : 1バイト符号なしデータ
            (rotationSpeedTmp, startAngleTmp) = struct.unpack_from("<2H", tmp)
            self.rpm = rotationSpeedTmp / 64
            startAngle = (startAngleTmp - 0xa000) / 64
            # 距離、強度データを格納する配列を用意
            distanceTmp = [0] * 8
            intensityTmp = [0] * 8
            tmpStart = 4
            tmpEnd = 6
            for i in range(8):
                tmp = data[tmpStart+(3*i):tmpEnd+(3*i)+1]
                (distanceTmp[i], intensityTmp[i]) = struct.unpack_from("<HB", tmp)

            tmp = data[28:29+1]
            endAngleTmp = struct.unpack_from("<H", tmp)
            endAngle = (endAngleTmp[0] - 0xa000) / 64

            return (distanceTmp, intensityTmp, startAngle, endAngle)

            # print("Send from ESP {0} ,packet size {1}".format(data,len(data)))

        
    def getData(self):
        preStartAngle = 0
        while True:
            (distanceTmp, intensityTmp, startAngle, endAngle) = self.getDataUnit()

            # 0度付近の場合は開始角度と終了角度の大小関係が逆になることがあるので、終了角度に360度足して大小関係を維持する
            if endAngle < startAngle:
                endAngle += 360

            # 開始角度が小さくなったら0度の場所なのでデータ更新フラグを立てる
            if (startAngle - preStartAngle < 0):
                self.dataObtained = True
            preStartAngle = startAngle

            # 角度をラジアンに変換
            startAngleRad = startAngle * math.pi / 180 * (-1 if self.isInvert else 1)
            endAngleRad = endAngle * math.pi / 180 * (-1 if self.isInvert else 1)
            # 1ステップ当たりの角度を計算
            angleIncrement = (endAngleRad - startAngleRad) / len(distanceTmp)
            # 排他制御開始
            self.lock.acquire()
            for i in range(len(distanceTmp)):
                self.theta[self.writePos] = startAngleRad + angleIncrement * i
                self.distance[self.writePos] = distanceTmp[i]
                self.intensity[self.writePos] = intensityTmp[i]
                self.writePos += 1
                if self.writePos >= self.dataSize:
                    self.writePos = 0
            # 排他制御終了
            self.lock.release()

    def run(self):
        self.thread.start()

    def stop(self):
        self.thread.stop()



if __name__ == "__main__":
    # 極座標のグラフを生成
    dist = plt.subplot(111, polar = True)
    # 表示する最大距離の初期値を2000に設定
    rmax = 2000

    capture = Capture(dataSize = 480, isInvert = True)
    capture.run()
    preTime = time.time()

    while True:
        if capture.dataObtained:
            # 描画を初期化
            plt.cla()

            # 排他制御開始
            capture.lock.acquire()
            # 距離データをプロット(散布図として描画)
            dist.scatter(list(capture.theta), list(capture.distance), c = "blue", s = 5)
            # 強度データをプロット(線でつなげて描画)
            dist.plot(list(capture.theta), list(capture.intensity), c = "orange", linewidth = 1)
            # データの描画をしたのでデータ取得フラグを下ろす
            capture.dataObtained = False
            # 排他制御開始
            capture.lock.release()

            # 画面の上を0度にする
            dist.set_theta_offset(math.pi / 2)
            # 表示する距離値の最大値を設定
            dist.set_rmax(rmax)
            # 描画処理
            plt.pause(0.01)
            # 現在設定されている表示最大距離値を取得
            rmax = dist.get_rmax()

        else:
            time.sleep(0.01)