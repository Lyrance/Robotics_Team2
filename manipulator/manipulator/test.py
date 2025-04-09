#!/usr/bin/env python3
import cv2
import numpy as np

def main():
    # 创建一幅空白图像（黑色背景）
    img = np.zeros((400, 600, 3), dtype=np.uint8)
    
    # 在图像上添加一些文本，便于确认显示是否正常
    cv2.putText(img, "OpenCV GUI Test", (50, 200),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    # 设置窗口名，并通过 namedWindow 创建窗口（可调节大小窗口）
    cv2.namedWindow("Test Window", cv2.WINDOW_NORMAL)
    
    # 显示图像到窗口
    cv2.imshow("Test Window", img)
    
    # 输出提示信息，并等待用户按下任意键关闭窗口
    print("请在弹出的窗口中按任意键关闭。")
    cv2.waitKey(0)
    
    # 关闭所有 OpenCV 窗口
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
