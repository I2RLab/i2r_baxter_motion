#!/usr/bin/env python
__author__ = 'behzad'
import rospy
import cv2
import numpy
from std_msgs.msg import (
    UInt8,
)
from i2r_baxter_motion.msg import (
    Human,
    TrustData,
    RobotData
)
import os
current_dir = os.path.dirname(__file__)
parrent_dir = os.path.abspath(os.path.join(current_dir, os.pardir))
img_dir = parrent_dir + '/img'
print (parrent_dir)



def draw_text(img, text, pos, size=2, color=(0, 0, 0), thickness=6, align=(0, 0)):
    dimensions, baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, size, thickness)
    cv2.putText(img, text, (pos[0] - int(dimensions[0]*align[0]), pos[1] + int(dimensions[1]*align[1])), cv2.FONT_HERSHEY_SIMPLEX, size, color, thickness, cv2.CV_AA)


def draw_overlay(img, text, pos=(0, 1080), width=1920, height=130, align=(0, 1), background=(0, 0, 0), size=4, color=(212, 242, 242), thickness=8):
    cv2.rectangle(img, (pos[0] - int(width*align[0]), pos[1] - int(height*align[1])), (pos[0] - int(width*(align[0] - 1)), pos[1] - int(height*(align[1] - 1))), background, -1)
    draw_text(img, text, (pos[0] + int(width/2), pos[1] - int(height/2)), size, color, thickness, align=(0.5, 0.5))


def line_test(img, value, colorline=0.8, message='Below {} line', color=(255, 255, 255)):
    if not isinstance(colorline, list):
        colorline = [colorline]
    for number, line in reversed(list(enumerate(colorline, start=1))):
        if value < line:
            draw_overlay(img, message, color=color)
            return True
    return False


def gui_draw():
    global emotion_show
    img = numpy.empty((1080, 1920, 3), numpy.uint8)
    img[:] = (149, 165, 166)
    bkcolor1= (198, 195, 193)
    bkcolor2= (220, 217, 215)
    bar_color1=(113, 204, 46)#(200, 247, 197)
    bar_color2=(219, 152, 52)
    bar_color3=(34, 126, 230)
    bar_bkcolor1=(149, 165, 166)#(100, 120, 100)#(149, 165, 166)
    bar_bkcolor2=(149, 165, 166)#(110, 75, 25)#(106,100,50)
    bar_bkcolor3=(149, 165, 166)#(110, 75, 25)#(106,100,50)
    bar_bkcolor4=(74, 82, 83)#(110, 75, 25)#(106,100,50)

    txt_color1=(48, 87, 20)#(100, 120, 100)#(149, 165, 166)
    txt_color2=(92, 64, 20)#(110, 75, 25)#(106,100,50)

    min_s=min(sr,sh)
    max_s=max(sr,sh)
    min_s //=  1
    max_s = max(max_s // 1 + 1, min_s+1)

    draw_bar(img, (100, 50), 'Robot Velocity', vel, color=bar_color1, bar_bkcolor=bar_bkcolor1, textcolor=txt_color1, bkcolor=bkcolor1)
    draw_bar(img, (100, 115), 'Robot Progress', sr, min=min_s, max=max_s, color=bar_color2, bar_bkcolor=bar_bkcolor2, textcolor=txt_color2, bkcolor=bkcolor2)
    draw_bar(img, (100, 180), 'Human Progress', max(sh, min_s), min=min_s, max=max_s, color=(80, 62, 44), bar_bkcolor=bar_bkcolor1, textcolor=txt_color1, bkcolor=bkcolor1)
    if trust_show:
        draw_bar(img, (100, 245), 'Trust', max(trust,0), color=bar_color3, bar_bkcolor=bar_bkcolor3, textcolor=txt_color2, bkcolor=bkcolor2)
        draw_bar(img, (100, 310), 'Robot Performance', max(pr, 0), color=(156, 188, 26), bar_bkcolor=bar_bkcolor1, textcolor=txt_color1, bkcolor=bkcolor1)
        draw_bar(img, (100, 375), 'Human Performance', ph, color=(182, 89, 155), bar_bkcolor=bar_bkcolor2, textcolor=txt_color2, bkcolor=bkcolor2)

    h, w = img.shape[:2]
    done = True
    if task_cur<num_trials:
        h1, w1 = task_img[task_cur % num_tasks].shape[:2]
        img[500:500+h1, int(w/6-w1/2):int(w/6-w1/2)+w1] = task_img[task_cur % num_tasks]
        draw_text(img, 'Current Task', (int(w/6-w1/2), 470), size=1.5, thickness=5)
        done = False
    else:
        h1, w1 = task_img[-1].shape[:2]
        img[500:500+h1, int(w/6-w1/2):int(w/6-w1/2)+w1] = task_img[-1]
        draw_text(img, 'Last Task', (int(w/6-w1/2), 470), size=1.5, thickness=5)
        if task_cur-num_trials<1:
            done = False

    if task_cur+1<num_trials:
        h1, w1 = task_img[(task_cur+1) % num_tasks].shape[:2]
        img[500:500+h1, int(5*w/6-w1/2):int(5*w/6-w1/2)+w1] = task_img[(task_cur+1) % num_tasks]
        draw_text(img, 'Next Task', (int(5*w/6-w1/2), 470), size=1.5, thickness=5)

    if emotion_show:
        h1, w1 = emotion_img[emotion].shape[:2]
        img[450:450+h1, int(3*w/6-w1/2):int(3*w/6-w1/2)+w1] = emotion_img[emotion]
        # draw_text(img, 'Baxter', (int(3*w/6-w1/2), 520), size=1.5, thickness=5)

    if extra_trust_show:
        h1, w1 = task_img[task_cur % num_tasks].shape[:2]
        draw_bar_vertical(img, (w/2-w1/4, 470), 'Trust', max(trust,0), width=w1/2, height=h1, color=bar_color3, bar_bkcolor=bar_bkcolor4)

    cv2.imshow('Trust', img)
    return done

def draw_bar(img, pos, title, value, min =0, max=1, x_offset=900, width=800, height=40, color=(212, 242, 242), bar_bkcolor=(106,100,50), size=1.5, textcolor=(0, 0, 0), thickness=4, bkcolor=None,):
    if bkcolor:
        cv2.rectangle(img, (0, pos[1]+15),
                      (1920, pos[1] - height-10), bkcolor, -1)
    cv2.rectangle(img, (pos[0]+x_offset, pos[1]+5), (pos[0]+x_offset+int(width*(value-min)/(max-min)), pos[1]-height), color, -1)
    # cv2.rectangle(img, (pos[0]+x_offset+int(width*(value-min)/(max-min)), pos[1]+5), (pos[0]+x_offset+width, pos[1]-height), bar_bkcolor, -1)
    cv2.rectangle(img, (pos[0]+x_offset, pos[1]+5), (pos[0]+x_offset+width, pos[1]-height), bar_bkcolor, 2)
    text = '{0:.2f}'.format(value)
    dimensions, baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, size, thickness)
    cv2.putText(img, text, (pos[0]+x_offset - int(dimensions[0])-5, pos[1] + int(dimensions[1]*0)), cv2.FONT_HERSHEY_SIMPLEX, size, textcolor, thickness, cv2.CV_AA)
    draw_text(img, title, pos, size, textcolor, thickness)

def draw_bar_vertical(img, pos, title, value, min =0, max=1, y_offset=30, width=300, height=400, color=(212, 242, 242), bar_bkcolor=(106,100,50), size=1.5, textcolor=(0, 0, 0), thickness=5):
    cv2.rectangle(img, (pos[0], pos[1]+height+y_offset), (pos[0]+width, pos[1]+y_offset+int(height*(max-value)/(max-min))), color, -1)
    cv2.rectangle(img, (pos[0], pos[1]+height+y_offset), (pos[0]+width, pos[1]+y_offset), bar_bkcolor, 2)
    text = '{0:.2f}'.format(value)
    dimensions, baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, size, thickness)
    cv2.putText(img, text, (pos[0]+width - int(dimensions[0])-5, pos[1] + int(dimensions[1]*0)), cv2.FONT_HERSHEY_SIMPLEX, size, textcolor, thickness, cv2.CV_AA)
    draw_text(img, title, pos, size, textcolor, thickness)

def get_robot(data):
    global vel, sr, sr_pre, task_cur
    vel = data.vel
    sr = data.sr
    if sr//.5-sr_pre//.5>0:
        task_cur += 1
    sr_pre = sr


def get_human(data):
    global sh
    sh = data.sh


def get_trust(data):
    global trust, pr, ph
    trust = data.trust
    pr = data.pr
    ph = data.ph

def get_emotion(data):
    global emotion
    emotion = data.data


def nothing(x):
    pass

def main():
    cv2.namedWindow('Trust', cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty('Trust', cv2.WND_PROP_FULLSCREEN, cv2.cv.CV_WINDOW_FULLSCREEN)
    global trust, vel, sr, sh, pr, ph, sr_pre, emotion
    global task_order, task_cur, num_tasks
    global trust_show, emotion_show, extra_trust_show
    global num_trials
    num_trials = 6
    num_tasks = 2
    task_cur = 0

    rospy.init_node('trust_monitor')
    global task_img
    task_img = [cv2.imread(img_dir + '/t1.png'), cv2.imread(img_dir + '/t2.png')]
    trust = 0.5
    vel = 0.0
    sr = 0
    sr_pre = 0
    sh = 0
    pr  = 0.5
    ph  = 0.5
    emotion = 0
    extra_trust_show = False
    try:
        extra_trust_show = rospy.get_param('~extra_trust_show')
    except KeyError:
        extra_trust_show = False
    try:
        trust_show = rospy.get_param('~trust_show')
    except KeyError:
        trust_show = False
    if trust_show:
        rospy.Subscriber('/trust/trust', TrustData, get_trust)

    emotion_show = False
    try:
        emotion_show = rospy.get_param('~emotion')
    except KeyError:
        emotion_show = False
    # emotion_show = True

    if emotion_show:
        rospy.Subscriber('/trust/emotion', UInt8, get_emotion)
        global emotion_img
        emotion_img_tmp = [cv2.imread(img_dir + '/UI_happy.png'), cv2.imread(img_dir + '/UI_worried.png'),
                           cv2.imread(img_dir + '/UI_bored.png')]
        emotion_img = []
        for img2 in emotion_img_tmp:
            rows, cols, channels = img2.shape
            img1 = numpy.empty((rows, cols, 3), numpy.uint8)
            # img1[:] = (214, 213, 210)
            img1[:] = (149, 165, 166)
            roi = img1[0:rows, 0:cols]
            img2gray = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
            ret, mask = cv2.threshold(img2gray, 10, 255, cv2.THRESH_BINARY)
            mask_inv = cv2.bitwise_not(mask)
            # Now black-out the area of logo in ROI
            img1_bg = cv2.bitwise_and(roi, roi, mask=mask_inv)
            # Take only region of logo from logo image.
            img2_fg = cv2.bitwise_and(img2, img2, mask=mask)
            # Put logo in ROI and modify the main image
            dst = cv2.add(img1_bg, img2_fg)
            img1[0:rows, 0:cols] = dst
            emotion_img.append(img1)

    rospy.Subscriber('/trust/robot_states', RobotData, get_robot)
    rospy.Subscriber('/trust/human_states', Human, get_human)

    r = rospy.Rate(10)  # 10hz
    key = cv2.waitKey(10) & 0xFF
    done = False
    while not rospy.is_shutdown() and key!=27 and not done:
        key = cv2.waitKey(10) & 0xFF
        # print(key)
        done = gui_draw()
        r.sleep()


if __name__ == '__main__':
    main()

