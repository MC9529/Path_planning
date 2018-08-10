import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as ani
import math
import sys

class Path_anim():
    def __init__(self, axis):
        self.path_img, = axis.plot([], [], color='c', linestyle='dashed', linewidth=0.15)

    def set_graph_data(self, x, y):
        self.path_img.set_data(x, y)

        return self.path_img, 

# 円を書く
def write_circle(center_x, center_y, angle, circle_size=0.2):#人の大きさは半径15cm
    # 初期化
    circle_x = [] #位置を表す円のx
    circle_y = [] #位置を表す円のy

    steps = 100 #円を書く分解能はこの程度で大丈夫
    for i in range(steps):
        circle_x.append(center_x + circle_size*math.cos(i*2*math.pi/steps))
        circle_y.append(center_y + circle_size*math.sin(i*2*math.pi/steps))
    
    circle_line_x = [center_x, center_x + math.cos(angle) * circle_size]
    circle_line_y = [center_y, center_y + math.sin(angle) * circle_size]
    
    return circle_x, circle_y, circle_line_x, circle_line_y

class Animation_robot():
    def __init__(self):
        self.fig = plt.figure()
        self.axis = self.fig.add_subplot(111)

    def fig_set(self):
        # 初期設定 軸
        MAX_x = 5
        min_x = -5
        MAX_y = 5
        min_y = -5

        self.axis.set_xlim(min_x, MAX_x)
        self.axis.set_ylim(min_y, MAX_y)

        # 軸
        self.axis.grid(True)

        # 縦横比
        self.axis.set_aspect('equal')

        # label
        self.axis.set_xlabel('X [m]')
        self.axis.set_ylabel('Y [m]')

    def plot(self, traj_x, traj_y): # ただのplot
        self.axis.plot(traj_x, traj_y)

        plt.show()

    def artist_anim_plot(self, traj_x, traj_y, traj_th, traj_paths, traj_g_x, traj_g_y):
        print('size = {0}'.format(len(traj_x)))
        imgs = []
        finish_buffa = 5# 終了後もそのまま数秒表示したいので

        for i in range(len(traj_x) + finish_buffa):
            img = []
            if i >= len(traj_x)-2:
                i = len(traj_x)-2
                img_text = self.axis.text(0.05, 0.9, 'step = {0}'.format(i), transform=self.axis.transAxes)
                img.append(img_text)
                img_goal_text = self.axis.text(0.35, 0.5, 'GOAL!!', transform=self.axis.transAxes, fontsize=30)
                img.append(img_goal_text)

            circle_x, circle_y, circle_line_x, circle_line_y = write_circle(traj_x[i], traj_y[i], traj_th[i], circle_size=0.2)

            robot_img = self.axis.plot(circle_x, circle_y, 'k')
            img.extend(robot_img)

            robot_angle_img = self.axis.plot(circle_line_x, circle_line_y, 'k')
            img.extend(robot_angle_img)

            traj_img = self.axis.plot(traj_x[:i], traj_y[:i], 'k', linestyle='dashed')
            img.extend(traj_img)
            
            # step数を追加
            img_text = self.axis.text(0.05, 0.9, 'step = {0}'.format(i), transform=self.axis.transAxes)
            img.append(img_text)
            # goalを追加
            img_goal = self.axis.plot(traj_g_x[i], traj_g_y[i], '*', color='b', markersize=15)
            img.extend(img_goal)
            # obstaclesを追加
            '''
            for k in range(obstacles.shape[0]):
                circle_x, circle_y = self.circle_make(obstacles[k, 0], obstacles[k, 1], obstacles[k ,2])
                img_obstacle = self.axis.plot(circle_x, circle_y, color='k')
                img.extend(img_obstacle)
            '''

            # pathを追加
            for k in traj_paths[i]:
                img_path = self.axis.plot(k.x, k.y, color='c', linewidth=0.15)
                img.extend(img_path)


            print('i = {0}'.format(i))
            imgs.append(img)

        animation = ani.ArtistAnimation(self.fig, imgs, blit=True)

        print('save_animation?')
        shuold_save_animation = int(input())

        if shuold_save_animation: 
            animation.save('basic_animation.gif', writer='imagemagick')

        plt.show()

    
    def func_anim_plot(self, traj_x, traj_y, traj_th, traj_paths, traj_g_x, traj_g_y):
        # selfにしておく
        self.traj_x = traj_x
        self.traj_y = traj_y
        self.traj_th = traj_th
        self.traj_paths = traj_paths
        self.traj_g_x = traj_g_x
        self.traj_g_y = traj_g_y

        # trajお絵かき
        self.traj_img, = self.axis.plot([], [], 'k', linestyle='dashed')

        # 円と向き
        self.robot_img, = self.axis.plot([], [], 'k')

        self.robot_angle_img, = self.axis.plot([], [], 'k')

        # goalを追加
        self.img_goal, = self.axis.plot([], [], '*', color='b', markersize=15)

        # dwa # 何本線引くかは考える
        # やり方分からない泣
        self.dwa_paths = []
        self.max_path_num = 100
        for k in range(self.max_path_num):
            self.dwa_paths.append(Path_anim(self.axis))
        
        # ステップ数表示
        self.step_text = self.axis.text(0.05, 0.9, '', transform=self.axis.transAxes)


        animation = ani.FuncAnimation(self.fig, self._update_anim, interval=100, \
                              frames=len(traj_g_x))

        plt.show()


    def _update_anim(self, i):
        # 全体
        self.dwa_imgs = []
        # DWApath用
        self.dwa_path_imgs = []

        self.traj_img.set_data(self.traj_x[:i+1], self.traj_y[:i+1])
        # 円を書く
        circle_x, circle_y, circle_line_x, circle_line_y = write_circle(self.traj_x[i], self.traj_y[i], self.traj_th[i], circle_size=0.2)

        self.robot_img.set_data(circle_x, circle_y)

        self.robot_angle_img.set_data(circle_line_x, circle_line_y)

        self.img_goal.set_data(self.traj_g_x[i], self.traj_g_y[i])

        count = 0
        # path_num = np.random.randint(0, len(self.traj_paths[i]), (1, 100))
        # print(path_num)
        
        for k in range(self.max_path_num):
            path_num = math.ceil(len(self.traj_paths[i])/(self.max_path_num)) * k
            
            if path_num >  len(self.traj_paths[i]) - 1:
                path_num = np.random.randint(0, len(self.traj_paths[i]))

            self.dwa_path_imgs.append(self.dwa_paths[k].set_graph_data(self.traj_paths[i][path_num].x, self.traj_paths[i][path_num].y))            

        self.step_text.set_text('step = {0}'.format(i))

        for img in [self.traj_img, self.robot_img, self.robot_angle_img, self.img_goal, self.step_text, self.dwa_path_imgs]:
            self.dwa_imgs.append(img)

        return self.dwa_imgs

'''

def func_anim_plot(self, traj_x, traj_y, traj_th, traj_paths, traj_g_x, traj_g_y):
        # selfにしておく
        self.traj_x = traj_x
        self.traj_y = traj_y
        self.traj_th = traj_th
        self.traj_paths = traj_paths
        self.traj_g_x = traj_g_x
        self.traj_g_y = traj_g_y

        # trajお絵かき
        self.traj_img, = self.axis.plot([], [], 'k', linestyle='dashed')

        # 円と向き
        self.robot_img, = self.axis.plot([], [], 'k')

        self.robot_angle_img, = self.axis.plot([], [], 'k')

        # goalを追加
        self.img_goal, = self.axis.plot([], [], '*', color='b', markersize=15)

        # dwa # 何本線引くかは考える
        # やり方分からない泣
        self.dwa_paths = []
        self.max_path_num = 50
        for k in range(self.max_path_num):
            self.dwa_paths.append(Path_anim(self.axis))
        
        # ステップ数表示
        self.step_text = self.axis.text(0.05, 0.9, '', transform=self.axis.transAxes)


        animation = ani.FuncAnimation(self.fig, self._update_anim, interval=100, \
                              frames=len(traj_g_x))

        plt.show()


    def _update_anim(self, i):
        # DWApath用
        self.dwa_imgs = []

        self.traj_img.set_data(self.traj_x[:i], self.traj_y[:i])
        # 円を書く
        circle_x, circle_y, circle_line_x, circle_line_y = write_circle(self.traj_x[i], self.traj_y[i], self.traj_th[i], circle_size=0.2)

        self.robot_img.set_data(circle_x, circle_y)

        self.robot_angle_img.set_data(circle_line_x, circle_line_y)

        self.img_goal.set_data(self.traj_g_x[i], self.traj_g_y[i])

        count = 0
        draw_num = math.ceil(len(self.traj_paths[i]) / (self.max_path_num-1)) 
        
        for k in range(len(self.traj_paths[i])):
            if k % draw_num == 0 and k > 0:
                self.dwa_imgs.append(self.dwa_paths[count].set_graph_data(self.traj_paths[i][k].x, self.traj_paths[i][k].y))
            
                count += 1

        self.step_text.set_text('step = {0}'.format(i))

        for img in [self.traj_img, self.robot_img, self.robot_angle_img, self.img_goal, self.step_text]:
            self.dwa_imgs.append(img)

        return self.dwa_imgs


'''