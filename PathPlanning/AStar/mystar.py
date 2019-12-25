import matplotlib.pyplot as plt
import math

"""
A_star

公式：f(n) = g(n)+h(n)
g(n)表示当前点到起点的距离，h(n)表示当前点到目标点的距离
维护两个列表，open列表表示待计算的点，close的表示已经搜索的点
算法过程：
1.将起始点放入open列表
2.重复以下步骤：
i.在open_list 查找F最小的点，并把查找点作为当前点
ii.把当前点从open_list 删除，添加到close_list 
iii. 对当前点相邻的点执行以下步骤：
    1. 如果该相邻点不可通行或已经在close_list，则跳过，继续下一节点
    2. 如果该相邻点不在open_list，则将该节点添加到open_list，并设置该相邻点的父节点为当前点，并保存G值和F值
    3. 如果该相邻点在open_list,则判断经由当前点到达该相邻节点的G值是否小于原来保存的G值，若小于，则将该相邻节点的父节点设为当前节点，并重新设置该相邻节点的G和F,因为H是不变的
iv. 循环结束条件
当终点节点被加入到open_list 作为待检验节点，表示路径已经找到
或者open_list为空，表明没有可以添加的节点，终点节点也没有被添加进来，所以表示路径查找失败
3. 从终点节点沿父节点遍历，并保存所有节点，遍历所得节点就是路径点


"""


show_animation = True

class Node:

    def __init__(self,x,y,cost,pind):
        self.x = x
        self.y = y
        self.cost = cost # 表示g(n),即到起始点的距离
        self.pind = pind # 父节点 的index

    def __str__(self):
        return str(self.x)+","+str(self.y)+","+str(self.cost)+","+str(self.pind)


def calc_obstacle_map(ox,oy,reso,vr):
    # reso 为分辨率
    # vr 机器半径

    print("[INFO] generating obstacle map")

    minx = round(min(ox))
    miny = round(min(oy))
    maxx = round(max(ox))
    maxy = round(max(oy))

    print("[INFO] minx:",minx)
    print("[INFO] miny:",miny)
    print("[INFO] maxx:",maxx)
    print("[INFO] maxy:",maxy)

    xwidth = round(maxx-minx)
    ywidth = round(maxy-miny)

    print("[INFO] xwidth:",xwidth)
    print("[INFO] ywidth:",ywidth)

    obmap = [[False for i in range(xwidth+1)] for i in range(ywidth+1)]

    # print("[INFO] obmap:",len(obmap[0]))

    # 根据具体环境，对障碍物作膨胀
    for ix in range(xwidth+1):
        x = ix + minx
        for iy in range(ywidth+1):
            y = iy + miny
            for iox,ioy in zip(ox,oy):
                d = math.hypot(iox-x,ioy-y)
                if d <= vr/reso:
                    obmap[ix][iy] = True
                    break
    
    return obmap,minx,miny,maxx,maxy,xwidth,ywidth


def verify_node(node,obmap,minx,miny,maxx,maxy):

    if node.x < minx or node.y < miny or node.x > maxx or node.y > maxy:
        return False
    
    if obmap[node.x][node.y]:
        return False

    return True

def calc_index(node,xwidth,xmin,ymin):

    return (node.y-ymin)*xwidth + (node.x-xmin) # 计算索引值

def get_motion_model():
    motion = [[1,0,1],
            [0,1,1],
            [-1,0,1],
            [0,-1,1],
            [-1,-1,math.sqrt(2)],
            [-1,1,math.sqrt(2)],
            [1,-1,math.sqrt(2)],
            [1,1,math.sqrt(2)]]

    return motion

def calc_heuristic(n1,n2):
    # 计算距离
    w = 1.0
    d = w*math.sqrt((n1.x-n2.x)**2 + (n1.y-n2.y)**2)

    return d

def calc_final_path(ngoal,closedset,reso):
    rx,ry = [ngoal.x*reso],[ngoal.y*reso]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(n.x*reso)
        ry.append(n.y*reso)

        pind = n.pind

    return rx,ry


def a_star_planning(sx,sy,gx,gy,ox,oy,reso,rr):
    
    nstart = Node(round(sx/reso),round(sy/reso),0.0,-1)
    ngoal = Node(round(gx/reso),round(gy/reso),0.0,-1)

    ox = [iox/reso for iox in ox]
    oy = [ioy/reso for ioy in oy]

    obmap,minx,miny,maxx,maxy,xw,yw = calc_obstacle_map(ox,oy,reso,rr)

    motion = get_motion_model()

    print("[INFO] motion: {0}".format(motion))

    openset,closeset = dict(),dict()

    openset[calc_index(nstart,xw,minx,miny)] = nstart

    while True:
        # 在open_list 查找F最小的点，并把查找点作为当前点
        c_id = min(openset,key=lambda o:openset[o].cost+calc_heuristic(ngoal,openset[o]))
        current = openset[c_id]

        if show_animation:
            plt.plot(current.x*reso,current.y*reso,"xc")
            if len(closeset.keys()) % 10 == 0:
                plt.pause(0.001)
        # plt.show()
        # break
        if current.x == ngoal.x and current.y == ngoal.y:
            print("find goall")
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break
        
        # 把当前点从open_list 删除，添加到close_list 
        del openset[c_id]
        closeset[c_id] = current

        for i in range(len(motion)):
            node = Node(current.x+motion[i][0], current.y+motion[i][1],
                        current.cost+motion[i][2],c_id)
            
            n_id = calc_index(node,xw,minx,miny)

            if n_id in closeset:
                continue

            if not verify_node(node,obmap,minx,miny,maxx,maxy):
                continue

            if n_id not in openset:
                openset[n_id] = node
            else:
                # 如果该相邻点在open_list,则判断经由当前点到达该相邻节点的G值是否小于原来保存的G值，
                # 若小于，则将该相邻节点的父节点设为当前节点，并重新设置该相邻节点的G和F
                if openset[n_id].cost > node.cost:
                    openset[n_id] = node

            # 以下代码也可用于更新node，但是逻辑不够清晰
            # tcost = current.cost + calc_heuristic(current,node)
            # if tcost >= node.cost:
            #     continue
            # node.cost = tcost
            # openset[n_id] = node
    
    rx,ry = calc_final_path(ngoal,closeset,reso)

    return rx,ry


                    






def main():


    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    grid_size = 2.0  # [m]
    robot_radius = 2.0  # [m]

    plt.ion()
    ox, oy = [], []
    for i in range(60):
        ox.append(i)
        oy.append(0.0)
    for i in range(60):
        ox.append(60.0)
        oy.append(i)
    for i in range(61):
        ox.append(i)
        oy.append(60.0)
    for i in range(61):
        ox.append(0.0)
        oy.append(i)

    for i in range(40):
        ox.append(20.0)
        oy.append(i)
    for i in range(40):
        ox.append(40.0)
        oy.append(60.0-i)

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")
    rx,ry = a_star_planning(sx, sy, gx, gy,ox,oy,grid_size,robot_radius)

    # plt.show()
    # plt.ioff()
    for x,y in zip(rx,ry):
        plt.plot(x,y,'o')
        # print(x,y)
        plt.pause(0.1)
    print('len ',len(rx))
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    main()