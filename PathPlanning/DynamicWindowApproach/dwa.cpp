    // dwaapp.cpp 
        //
        // #include "pch.h"
        #include <iostream>
        #include <math.h>
        #include<algorithm>
        #include <opencv2/core/core.hpp>  
        #include <opencv2/highgui/highgui.hpp> 
        #include <opencv2/imgproc/imgproc.hpp>
        #include <stdlib.h>
        #include <time.h>
        using namespace cv;
        using namespace std;
        #define pi 3.1415926535897932384626433832795
        //下标宏定义 状态[x(m), y(m), yaw(Rad), v(m / s), w(rad / s)]
        #define POSE_X          0    // 坐标 X
        #define POSE_Y          1    // 坐标 Y
        #define YAW_ANGLE       2    // 机器人航向角
        #define V_SPD           3    // 机器人速度
        #define W_ANGLE_SPD     4    // 机器人角速度
        //定义Kinematic的下标含义
        #define MD_MAX_V        0    // 最高速度m / s]
        #define MD_MAX_W        1    // 最高旋转速度[rad / s]
        #define MD_ACC          2    // 加速度[m / ss]
        #define MD_VW           3    // 旋转加速度[rad / ss]
        #define MD_V_RESOLUTION 4    // 速度分辨率[m / s]
        #define MD_W_RESOLUTION 5    // 转速分辨率[rad / s]]
        struct state
        {
        	float x;
        	float y;
        	float yaw;
        	float velocity;
        	float angular;
        };
        struct controlU
        {
        	float vt;
        	float wt;
        };
        struct maxmotion
        {
        	float minvel;
        	float maxvel;
        	float minang;
        	float maxang;
        };
        struct eval_db
        {
        	float vt;
        	float wt;
        	float heading;
        	float dist;
        	float vel;
        	float feval;
        };
        float dt = 0.1;//时间[s]`在这里插入代码片`
        /****************************************************/
        /*
        % degree to radian
        */
        /****************************************************/
        float DegreeToRadian(float degree)
        {
        	return degree / 180 * pi;
        }
        /****************************************************/
        /*
        % radian to degree
        */
        /****************************************************/
        float RadianToDegree(float radian)
        {
        	return radian / pi * 180;
        }
        //模拟区域范围 [xmin xmax ymin ymax]
        //float area[4] = { -1, 11, -1, 11 };
        /****************************************************/
        /*
        % Motion Model 根据当前状态推算下一个控制周期（dt）的状态
        % u = [vt; wt]; 当前时刻的速度、角速度 x = 状态[x(m), y(m), yaw(Rad), velocity(m / s), angular(rad / s)]
        */
        /****************************************************/
        state CarState(state cx, controlU u)
        {
        	state result;
        	result.x = cx.x + dt * cos(cx.yaw)*u.vt;
        	result.y = cx.y + dt * sin(cx.yaw)*u.vt;
        	result.yaw = cx.yaw + dt * u.wt;
        	result.velocity = u.vt;
        	result.angular = u.wt;
        	return result;
        }
        /****************************************************/
        /*
        % 计算动态窗口
        % 返回 最小速度 最大速度 最小角速度 最大角速度速度
        */
        /****************************************************/
        maxmotion CalcDynamicWindow(state cx, float *model)
        {
        	maxmotion V_r;
        	//车子速度的最大最小范围 依次为：最小速度 最大速度 最小角速度 最大角速度速度
        	//float Vs[4] = { 0, model[MD_MAX_V], -model[MD_MAX_W], model[MD_MAX_W] };
        	//根据当前速度以及加速度限制计算的动态窗口  依次为：最小速度 最大速度 最小角速度 最大角速度速度
        	//float Vd[4] = { cx.velocity - model[MD_ACC]*dt, cx.velocity + model[MD_ACC]*dt, cx.angular - model[MD_VW]*dt, cx.angular + model[MD_VW]*dt };
        	//最终的Dynamic Window
        	//float Vtmp[2 * 4];// 2 X 4  每一列依次为：最小速度 最大速度 最小角速度 最大角速度速度
        	//memcpy(&Vtmp,&Vs,4*sizeof(float));
        	//memcpy(&Vtmp[4], &Vd, 4 * sizeof(float));
        	//V_r.minvel = max(Vs[0], Vd[0]);
        	//V_r.maxvel = min(Vs[1], Vd[1]);
        	//V_r.minang = max(Vs[3], Vd[3]);
        	//V_r.maxang = min(Vs[4], Vd[4]);
        	V_r.minvel = max(0.0f, cx.velocity - model[MD_ACC] * dt);
        	V_r.maxvel = min(model[MD_MAX_V], cx.velocity + model[MD_ACC] * dt);
        	V_r.minang = max(-model[MD_MAX_W], cx.angular - model[MD_VW] * dt);
        	V_r.maxang = min(model[MD_MAX_W], cx.angular + model[MD_VW] * dt);
        	return V_r;
        }
        /****************************************************/
        /*
        % heading的评价函数计算
        % 输入参数：当前位置、目标位置
        % 输出参数：航向参数得分  当前车的航向和相对于目标点的航向 偏离程度越小 分数越高 最大180分
        */
        /****************************************************/
        float CalcHeadingEval(state cx, Point goal)
        {
        	float theta = RadianToDegree(cx.yaw); //机器人朝向
        	float goalTheta = RadianToDegree(atan2(goal.y - cx.y, goal.x - cx.x));   //目标点相对于机器人本身的方位 
        	float targetTheta;
        	if (goalTheta > theta)
        		targetTheta = goalTheta - theta; //[deg]
        	else
        		targetTheta = theta - goalTheta; //[deg]
        	return 180 - targetTheta;
        }
        /****************************************************/
        /*
        % 障碍物距离评价函数  （机器人在当前轨迹上与最近的障碍物之间的距离，如果没有障碍物则设定一个常数）
        % 输入参数：位姿、所有障碍物位置、障碍物半径
        % 输出参数：当前预测的轨迹终点的位姿距离所有障碍物中最近的障碍物的距离 如果大于设定的最大值则等于最大值
        % 距离障碍物距离越近分数越低
        */
        /****************************************************/
        float CalcDistEval(state cx, vector<Point> ob, float R)
        {
        	float dist = 100.0;
        	for (int i = 0; i < ob.size(); ++i)
        	{
        		//到第i个障碍物的距离 - 障碍物半径
        		float disttmp = sqrt((ob[i].x - cx.x)*(ob[i].x - cx.x) + (ob[i].y - cx.y)*(ob[i].y - cx.y)) - R;
        		if (dist > disttmp)//大于最小值 则选择最小值
        			dist = disttmp;
        	}
        	if (dist >= 2 * R)
        		dist = 2 * R;
        	return dist;
        }
        /****************************************************/
        /*
        % 计算制动距离
        % 根据运动学模型计算制动距离, 也可以考虑成走一段段圆弧的累积 简化可以当一段段小直线的累积
        */
        /****************************************************/
        float CalcBreakingDist(float vel, float mdacc)
        {
        	float stopDist = 0;
        	while (vel > 0)//给定加速度的条件下 速度减到0所走的距离
        	{
        		stopDist = stopDist + vel * dt; //制动距离的计算
        		vel = vel - mdacc*dt;
        	}
        	return stopDist;
        }
        /****************************************************/
        /*
        % 单条轨迹生成、轨迹推演函数
        % 输入参数： 当前状态、vt当前速度、ot角速度、evaldt 前向模拟时间、机器人模型参数（没用到）
        % 返回参数;
        %           x   : 机器人模拟时间内向前运动 预测的终点位姿(状态);
        %           traj: 当前时刻 到 预测时刻之间 过程中的位姿记录（状态记录） 当前模拟的轨迹
        %                  轨迹点的个数为 evaldt / dt + 1 = 3.0 / 0.1 + 1 = 31
        */
        /****************************************************/
        state GenerateTrajectory(state cx, vector<state> *traj, float vt, float wt, float evaldt, float *model)
        {
        	float time = 0.0;
        	controlU u = { vt, wt };
        	traj->clear();
        	traj->push_back(cx);
        	state px = cx;
        	while ((int)time < (int)evaldt*10)
        	{
        		time = time + 10*dt;
        		px = CarState(px, u);
        		traj->push_back(px);
        	}
        	return px;
        }
        /****************************************************/
        /*
        % 评价函数 内部负责产生可用轨迹
        % 输入参数 ：当前状态、参数允许范围（窗口）、目标点、障碍物位置、障碍物半径、评价函数的参数
        % 返回参数：
        %           evalDB N * 5  每行一组可用参数 分别为 速度、角速度、航向得分、距离得分、速度得分
        %           trajDB      每5行一条轨迹 每条轨迹包含 前向预测时间 / dt + 1 = 31 个轨迹点（见生成轨迹函数）
        */
        /****************************************************/
        void Evaluation(state cx, vector<eval_db> *EvalDb, vector<state> *TrajDb, maxmotion Vr, Point goal, vector<Point> ob, float R, float *model, float evaldt)
        {
        	EvalDb->clear();
        	TrajDb->clear();
        	vector<state> traj;
        	for (float vt = Vr.minvel; vt <= Vr.maxvel; vt = vt + model[4])//根据速度分辨率遍历所有可用速度： 最小速度和最大速度 之间 速度分辨率 递增
        	{
        		for (float wt = Vr.minang; wt <= Vr.maxang; wt = wt + model[5])//根据角度分辨率遍历所有可用角速度： 最小角速度和最大角速度 之间 角度分辨率 递增  
        		{
        			//轨迹推测; 得到 xt : 机器人向前运动后的预测位姿; traj: 当前时刻 到 预测时刻之间的轨迹（由轨迹点组成）
        			state xt = GenerateTrajectory(cx, &traj, vt, wt, evaldt, model); //evaldt = evalParam(4), 前向模拟时间;
        			//各评价函数的计算
        			float heading = CalcHeadingEval(xt, goal);//前项预测终点的航向得分  偏差越小分数越高
        			float dist = CalcDistEval(xt, ob, R);//前项预测终点 距离最近障碍物的间隙得分 距离越远分数越高
        			float vel = fabs(vt);//速度得分 速度越快分越高
        			float stopDist = CalcBreakingDist(vel, model[MD_ACC]); //制动距离的计算
        			eval_db db = { vt,wt,heading,dist,vel };
        			if (dist > stopDist)
        			{
        				EvalDb->push_back(db);
        				//TrajDb->insert(TrajDb->end(), traj.begin(), traj.end());////?????????
        			}
        		}
        	}
        }
        /****************************************************/
        /*
        % 归一化处理
        % 每一条轨迹的单项得分除以本项所有分数和
        */
        /****************************************************/
        void NormalizeEval(vector<eval_db> *EvalDb)
        {
        	//评价函数正则化
        	float sum3 = 0, sum4 = 0, sum5 = 0;
        	for (int i = 0; i < EvalDb->size(); ++i)
        	{
        		sum3 += EvalDb->at(i).heading;
        		sum4 += EvalDb->at(i).dist;
        		sum5 += EvalDb->at(i).vel;
        	}
        	if (sum3 != 0)
        	{
        		for (int i = 0; i < EvalDb->size(); ++i)
        			EvalDb->at(i).heading = EvalDb->at(i).heading / sum3;
        	}
        	if (sum4 != 0)
        	{
        		for (int i = 0; i < EvalDb->size(); ++i)
        			EvalDb->at(i).dist = EvalDb->at(i).dist / sum4;
        	}
        	if (sum5 != 0)
        	{
        		for (int i = 0; i < EvalDb->size(); ++i)
        			EvalDb->at(i).vel = EvalDb->at(i).vel / sum5;
        	}
        }
        /****************************************************/
        /*
        % DWA算法实现
        % model  机器人运动学模型  最高速度m/s],最高旋转速度[rad/s],加速度[m/ss],旋转加速度[rad/ss], 速度分辨率[m/s],转速分辨率[rad/s]]
        % 输入参数：当前状态、模型参数、目标点、评价函数的参数、障碍物位置、障碍物半径
        % 返回参数：控制量 u = [v(m/s),w(rad/s)] 和 轨迹集合 N * 31  （N：可用的轨迹数）
        % 选取最优参数的物理意义：在局部导航过程中，使得机器人避开障碍物，朝着目标以较快的速度行驶。
        */
        /****************************************************/
        controlU DynamicWindowApproach(state cx, vector<state> *TrajDb, float *model, Point goal, float * evalParam, vector<Point> ob, float R)
        {
        	controlU u;
        	vector<eval_db> EvalDb;
        	// Dynamic Window [vmin,vmax,wmin,wmax] 最小速度 最大速度 最小角速度 最大角速度速度
        	maxmotion cvr = CalcDynamicWindow(cx, model); //根据当前状态 和 运动模型 计算当前的参数允许范围
        	//评价函数的计算 evalDB N*5  每行一组可用参数 分别为 速度、角速度、航向得分、距离得分、速度得分
        	//trajDB      每5行一条轨迹 每条轨迹都有状态x点串组成
        	//Evaluation(state cx, vector<state> *traj, vector<eval_db> *EvalDb, vector<state> *TrajDb, maxmotion Vr, Point goal, vector<Point> ob, float R, float *model, float evaldt)
        	Evaluation(cx, &EvalDb, TrajDb,cvr, goal, ob, R, model, evalParam[3]);  //evaldt = evalParam(4) 评价函数参数[heading, dist, velocity, predictDT]
        	if (EvalDb.empty())
        	{
        		cout << "no path to goal!!" << endl;
        		u.vt = 0;
        		u.wt = 0;
        		return u;
        	}
        	NormalizeEval(&EvalDb);//各评价函数正则化
        	//最终评价函数的计算float heading;float dist;float vel;
        	for (int i = 0; i < EvalDb.size(); ++i)
        		EvalDb.at(i).feval = evalParam[0] * EvalDb.at(i).heading + evalParam[1] * EvalDb.at(i).dist + evalParam[2] * EvalDb.at(i).vel;//根据评价函数参数 前三个参数分配的权重 计算每一组可用的路径参数信息的得分
        	float maxheading = EvalDb.at(0).feval;
        	float idx = 0;
        	for (int i = 0; i < EvalDb.size(); ++i)
        	{
        		if (maxheading < EvalDb.at(i).feval)
        		{
        			maxheading = EvalDb.at(i).feval;
        			idx = i;
        		}
        	}
        	u.vt = EvalDb.at(idx).vt;
        	u.wt = EvalDb.at(idx).wt;
        	return u;
        }
        /****************************************************/
        /*
        % 坐标转换
        % 输入参数：cx 机器人当前状态
        % 返回参数：机器人在图像中显示的状态
        						  x
        		  ----------------->
        		  |      /|\
        		  |     x0|       Z:-90   [1 0  [0 -1 [x  X = -y0+width/2
        		  |   y0  |       X:180    0 -1] 1  0] y] Y = -x + height/2
        		y | <------
        		 \|/
        
        */
        /****************************************************/
        state toshowcoodinate(state cx, float width, float height)
        {
        	state sx = { 0,0,0,0,0 };
        	sx.x = -cx.y + width / 2;
        	sx.y = -cx.x + height / 2;
        	sx.yaw = -cx.yaw - pi / 2;
        	return sx;
        }
        /****************************************************/
        /*
        % 坐标转换
        % 输入参数：point 坐标
        % 返回参数：在图像中显示的坐标
        */
        /****************************************************/
        Point pointtoshowcoodinate(Point cx, float width, float height)
        {
        	Point sx = { 0,0 };
        	sx.x = -cx.y + width / 2;
        	sx.y = -cx.x + height / 2;
        	return sx;
        }
        /****************************************************/
        /*
        % 在img绘制机器人
        % 输入参数：carstate 机器人状态
        */
        /****************************************************/
        void showcar(Mat & img, state carstate )
        {
        	float R = 15.0;
        	int thickness = 2;//-1为实心圆
        	int lineType = 8;
        	Point End;
        	state draw = toshowcoodinate(carstate, img.cols, img.rows);
        	End.x = draw.x + cos(draw.yaw)*R;
        	End.y = draw.y + sin(draw.yaw)*R;
        	circle(img,
        		Point(draw.x, draw.y),//圆心由点center定义
        		(int)R,//圆的半径
        		Scalar(0, 255, 0), //圆的颜色
        		thickness,//线粗
        		lineType);
        	line(img, Point(draw.x, draw.y), End, Scalar(0, 0, 255), thickness, lineType);
        }
        
        /****************************************************/
        /*
        % 绘制所有障碍物位置
        % 输入参数：obstacle 所有障碍物的坐标   obstacleR 障碍物的半径
        */
        /****************************************************/
        void DrawObstacle_plot(Mat& img, vector<Point> ob, float obstacleR)
        {
        	int thickness = 1;//-1为实心圆
        	int lineType = 8;
        	float theta = 0;
        	for (int i = 0; i < ob.size(); ++i)
        	{
        		Point pob = pointtoshowcoodinate(ob.at(i), img.cols, img.rows);
        		//pob.x = -ob.at(i).y + img.cols / 2;
        		//pob.y = -ob.at(i).x + img.rows / 2;
        		circle(img, pob, obstacleR, Scalar(0, 0, 0), thickness, lineType);
        		//plot....绘制所有障碍物位置
        	}
        }
        /****************************************************/
        /*
        % 在图像img画箭头
        % 输入参数：pStart 起点   pEnd 终点 len箭头长度，alpha 箭头开叉角度，color 箭头颜色
        */
        /****************************************************/
        void drawArrow(Mat& img, Point pStart, Point pEnd, float len, float alpha, Scalar color, int thickness, int lineType)
        {
        	//float alpha = 15.0;//箭头角度
        	Point arrow;
        	//计算 θ 角（最简单的一种情况在下面图示中已经展示，关键在于 atan2 函数，详情见下面）   
        	double angle = atan2((double)(pStart.y - pEnd.y), (double)(pStart.x - pEnd.x));
        	line(img, pStart, pEnd, color, thickness, lineType);
        	//计算箭角边的另一端的端点位置（上面的还是下面的要看箭头的指向，也就是pStart和pEnd的位置） 
        	arrow.x = pEnd.x + len * cos(angle + pi * alpha / 180);
        	arrow.y = pEnd.y + len * sin(angle + pi * alpha / 180);
        	line(img, pEnd, arrow, color, thickness, lineType);
        	arrow.x = pEnd.x + len * cos(angle - pi * alpha / 180);
        	arrow.y = pEnd.y + len * sin(angle - pi * alpha / 180);
        	line(img, pEnd, arrow, color, thickness, lineType);
        }
        /****************************************************/
        /*
        % 在图像img画箭头
        % 输入参数：pStart 起点   pEnd 终点 len箭头长度，alpha 箭头开叉角度，color 箭头颜色
        */
        /****************************************************/
        void drawtraj(Mat& img, vector<state> traj)
        {
        	int thickness = 3;//-1为实心圆
        	int lineType = 8;
        	state pLast = toshowcoodinate(traj.at(0), img.cols, img.rows);;
        	for (int i = 0; i < traj.size(); ++i)
        	{
        		state pEnd = toshowcoodinate(traj.at(i), img.cols, img.rows);
        		line(img, Point(pLast.x, pLast.y), Point(pEnd.x, pEnd.y), Scalar(255, 0, 0), thickness, lineType);
        		pLast = pEnd;
        	}
        }
        
        void DwaSample()
        {
        	cout << "Dynamic Window Approach sample program start!!" << endl;
        	state statex = { -300,0,pi/10,0,0 };//机器人初始状态
        	vector<state> cartraj,realtraj; //cartraj累积存储走过的轨迹点的状态值
        	//机器人运动学模型参数, 最高速度m / s], 最高旋转速度[rad / s], 加速度[m / ss], 旋转加速度[rad / ss],速度分辨率[m / s], 转速分辨率[rad / s]]
        	float Kinematic[6] = { 10.0, DegreeToRadian(20.0), 2, DegreeToRadian(50.0), 0.1, DegreeToRadian(1.0) };
        	Point end(500, -500);
        	//评价函数参数 [heading,dist,velocity,predictDT],航向得分的比重、距离得分的比重、速度得分的比重、向前模拟轨迹的时间
        	float evalParam[4] = { 0.05, 0.2, 0.1, 3.0 };
        	//障碍物位置列表[x(m) y(m)]
        	vector <Point> obstacle = { {0, 100 }, {100, 100},{100, 250},{200, 100},{250, 200},{250, 250},{250, 300},{250, 350},{250, 450},{400, 400},{400, 450},{350, 450} };
        	//vector <Point> obstacle = { {0, 20 }, {20, 40},{20, 50},{40, 20},{50, 40},{50, 50},{50, 60},{50, 70},{50, 90},{80, 80},{80, 90},{70, 90} };
        	float obstacleR = 15; //冲突判定用的障碍物半径
        	bool is_end = false;
        	int count = 0;
        	while (!is_end)
        	{
        		//DWA参数输入 返回控制量 u = [v(m/s),w(rad/s)] 和 轨迹
        		controlU cu = DynamicWindowApproach(statex, &cartraj, Kinematic, end, evalParam, obstacle, obstacleR);
        		statex = CarState(statex, cu);//cu 速度控制量，机器人移动到下一个时刻的状态量 根据当前速度和角速度推导 下一刻的位置和角度
        		//历史轨迹的保存
        		realtraj.push_back(statex);//最新结果 以列的形式 添加到轨迹中
        		float tdist = sqrt((statex.x - end.x)*(statex.x - end.x) + (statex.y - end.y)*(statex.y - end.y));
        		if (tdist < 0.5)//是否到达目标点
        		{
        			cout << "Arrive Goal!" << endl;
        			is_end = true;
        		}
        		count++;
        		/*if (count % 200 == 0)
        		{
        			end.x = 5*(rand()%100);
        			end.y = 5 * (rand() % 100);
        		}*/
        		printf("count %d\n",count);
        		//state showstate = toshowcoodinate(cstate, img.cols, img.rows);
        		Mat img(Size(1000, 1000), CV_8UC3);
        		showcar(img, statex);
        		DrawObstacle_plot(img, obstacle, obstacleR);
        		drawtraj(img, realtraj);
        		//图片5倍放大
        		double scale = 1.0;
        		Size dsize = Size(img.cols*scale, img.rows*scale);
        		Mat image = Mat(dsize, CV_8UC3);
        		resize(img, image, dsize);
        		img.release();
        		// 创建 "dwa"窗口    
        		namedWindow("dwa");
        		// 显示   
        		imshow("dwa", image);
        		waitKey(1);
        	}
        }
        
        int main()
        {
        	srand((unsigned)time(NULL));
        	DwaSample();
        	waitKey(1);
        	return 0;
        }
