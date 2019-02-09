#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import video

# 定义常量，生成颜色模板时需要用到
LOWER_BLUE = np.array([0., 60., 32.])  # 蓝色的下界
UPPER_BLUE = np.array([180., 255., 255.])  # 蓝色的上界



class App(object):
    def __init__(self, color):
        self.cam = video.create_capture(0)  # 捕获摄像头设备并创建一个对象
        self.frame = None
        cv2.namedWindow('camshift')  # cv2窗口对话框名称
        self.hist_roi = None
        self.selection = None
        self.tracking_state = 0
        self.hide_background = False  # 是否需要隐藏背景，默认显示

        if color == 'blue':
            self.flag = 'blue'
            self.roi = cv2.imread('blue.jpg')  # 读取blue.jpg作为感兴趣的区域

    # 初始化状态参数
    def start(self):
        self.selection = (0, 0, 640, 480)  # 选取该区域作为颜色识别检测区域
        self.tracking_state = 1  # 是否需要跟踪检测

    def get_mask(self, hsv_image, color='blue'):
        # 获得hsv_image对应颜色的蒙板
        if color not in ['blue', 'green', 'red']:
            return cv2.inRange(hsv_image, np.array([0., 0., 0.]), 
                               np.array([255., 255., 255.]))
        elif color == 'blue':
            return cv2.inRange(hsv_image, LOWER_BLUE, UPPER_BLUE)


    def show_hist(self):
        # 展示图片的直方图
        bin_count = self.hist_roi.shape[0]
        bin_w = 24
        img = np.zeros((256, bin_count * bin_w, 3), np.uint8)
        for i in xrange(bin_count):
            h = int(self.hist_roi[i])
            cv2.rectangle(img, (i * bin_w + 2, 255), ((i + 1) * bin_w - 2, 255 - h),
                          (int(180.0 * i / bin_count), 255, 255), -1)
        img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        cv2.imshow('hist', img)

    def run(self):
        roi = self.roi  # 获取ROI
        self.start()
        while True:
            ret, self.frame = self.cam.read()  # 调用摄像头读取图像
            vis = self.frame.copy()  # 创建相机图像副本
            
            # 将当前帧从RGB格式转换为HSV格式
            hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV) 

            # 获得当前hsv图像的掩模
            mask = self.get_mask(hsv, color=self.flag)

            if self.selection:
                x0, y0, x1, y1 = self.selection
                self.track_window = (x0, y0, x1, y1)  # 追踪子区域

                # 对ROI进行颜色格式转换和阈值限制
                hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                mask_roi = self.get_mask(hsv_roi, color=self.flag)
                # 绘制ROI图像的一维直方图
                hist_roi = cv2.calcHist([hsv_roi], [0], mask_roi, [16], [0, 180])
                # 对hist做归一化
                cv2.normalize(hist_roi, hist_roi, 0, 255, cv2.NORM_MINMAX)

                # 将hist向量reshape为1列并存入self.hist中
                self.hist_roi = hist_roi.reshape(-1)
                self.show_hist()
                
                # 可见区域
                # 目标图的待搜索区域;注意此时x,y的顺序
                # 像素坐标系:x轴向右,y轴向下,即x轴对应图像宽度,有多少列像素,y轴对应图像高度，代表行
                vis_roi = vis[y0:y1, x0:x1] 
                cv2.bitwise_not(vis_roi, vis_roi)  # 对每个像素进行二进制取反操作

                # 在vis中，置mask中为0的对应位置也为0
                vis[mask == 0] = 0

            if self.tracking_state == 1:
                self.selection = None  # 取消ROI模板
                
                # 反向投影法
                prob = cv2.calcBackProject([hsv], [0], self.hist_roi, [0, 180], 1)  
                prob &= mask  # 与mask进行与运算 得到所求颜色的直方图概率分布
                
                # CamShift算法迭代终止条件：达到最大迭代次数或者达到收敛阈值
                criteria_term = (cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT, 10, 1)  
                # camshift算法根据反向投影图计算目标颜色的质心，实现对目标颜色的跟踪；
				# 同时返回搜索窗信息，用于下一次迭代中调整搜索窗的位置和大小
				# track_box存储搜索窗的状态信息(圆心坐标，长/短轴，角度)
                track_box, self.track_window = cv2.CamShift(prob, self.track_window, 																criteria_term)

                if track_box[1][1] <= 1:
                    # 如果没有检测到 重置检测状态
                    self.start()
                else:
                    # 检测到目标颜色
                    if self.hide_background:
                        # 如果需要隐藏背景， 使用prob直方概率分布图替换vis图像
                        vis[:] = prob[..., np.newaxis]
                    try:
                        '''
                        track_box: [[center, axes], [angle, startAngle], endAngle]
                        '''
                        # 在track_box内部绘制椭圆图像
                        # 设置搜索窗的属性：(0,255,255)在BGR空间中为黄色,2为椭圆线圈像素宽度
                        a = str(track_box[0][0])+" "+str(track_box[0][1])+
                            " "+str(round(track_box[1][0],2))\+
                            " "+str(round(track_box[1][1],2))+
                            " "+str(round(track_box[2],2))+"\r\n"
                        print a
                    except:
                        print track_box

            cv2.imshow('camshift', vis)

            ch = 0xFF & cv2.waitKey(5)  # 保留返回值的低8位
            if ch == 27 or ch == ord('q'):  # 27对应ESC，即按ESC键退出
                break
            if ch == ord('b'):  # 输入b改变是否需要显示背景
                self.hide_background = not self.hide_background
            if ch == ord('r'):   # 重新开始检测颜色
                self.start()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    import sys

    try:
        print sys.argv[1]
        color = sys.argv[1]
    except IndexError:
        # 命令行参数未指定检测的颜色
        color = 'blue'

    a = App(color)
    a.run()