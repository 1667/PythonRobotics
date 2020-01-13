#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <iostream>
#include <iomanip>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>

#include <sys/time.h>

using namespace std;
using namespace Eigen;

#define SHOW_DEBUG 0
const double DERIV_STEP = 1e-6;
const int MAX_ITER = 100;

// From https://blog.csdn.net/stihy/article/details/52737723


// double func(const VectorXd& input,const VectorXd& output,const VectorXd& params,double objIndex)
// {
//     double x1 = params(0);
//     double x2 = params(1);
//     double x3 = params(2);
//     double x4 = params(3);

//     double t = input(objIndex);
//     double f = output(objIndex);

//     return x1 * sin(x2*t)+x3 * cos(x4*t)-f;

// }

double func(const VectorXd& input, VectorXd& output, const VectorXd& params,double objIndex,bool issub=true)
{
    double x1 = params(0);
    double x2 = params(1);
    double x3 = params(2);
    double x4 = params(3);

    double t = input(objIndex);
    double f = output(objIndex);

    double tmpf = x1 * sin(x2*t)+x3 * cos(x4*t);

    // double tmpf = x1*sin(t) * +(x2*t)+x3*pow(t,3) * cos(x4*t);

    if(issub)
    {
        return tmpf-f;
    }
    else{
        return tmpf;   
    }

}

//计算 误差量
VectorXd objF(const VectorXd& input,VectorXd& output,const VectorXd& params)
{
    VectorXd obj(input.rows());

    for(int i=0;i<input.rows();i++)
    {
        obj(i) = func(input,output,params,i);

    }
    return obj;

}

// F = (f^t*f)/2
double Func(const VectorXd& obj)
{
    return obj.squaredNorm()/2;
}
//数值求导法，求导
double Deriv(const VectorXd& input,VectorXd& output,int objIndex,const VectorXd& params,int paraIndex)
{
    VectorXd para1 = params;
    VectorXd para2 = params;
    para1(paraIndex) -= DERIV_STEP;
    para2(paraIndex) += DERIV_STEP;

    double obj1 = func(input,output,para1,objIndex);
    double obj2 = func(input,output,para2,objIndex);
    
    return (obj2-obj1)/(2*DERIV_STEP);

}

MatrixXd Jacobin(const VectorXd& input,VectorXd& output,const VectorXd& params)
{
    int rowNum = input.rows();
    int colNum = params.rows();

    MatrixXd Jac(rowNum,colNum);

    for(int i=0;i<rowNum;i++)
    {
        for(int j=0;j<colNum;j++)
        {
            Jac(i,j) = Deriv(input,output,i,params,j);

        }
    }

    return Jac;

}

void gaussNewton(const VectorXd& input,VectorXd& output, VectorXd& params)
{
    int errNum = input.rows();
    int paraNum = params.rows();

    VectorXd obj(errNum);

    double last_sum = 0;
    int iterCnt = 0;

    while (iterCnt < MAX_ITER)
    {
        obj = objF(input,output,params);
        
        double sum = 0;
        sum = Func(obj);

        if(SHOW_DEBUG)
        {
            cout << "Iterator index: " << iterCnt << endl;
            cout << "parameters: " <<endl << params << endl;
            cout << "error sum: " << endl << sum << endl << endl;
        }
        
        
        if (fabs(sum - last_sum) <= 1e-12)
        {
            break;
        }
        last_sum = sum;

        MatrixXd Jac = Jacobin(input,output,params);
        
        VectorXd delta(paraNum);
        delta = (Jac.transpose()*Jac).inverse()*Jac.transpose()*obj;

        params -= delta;
        iterCnt++;

    }
    
    cout << "GS "<< iterCnt <<endl;

}

double maxMatrixDiagonale(const MatrixXd& Hessian)
{ 
    int max = 0;
    for(int i = 0; i < Hessian.rows(); i++)
    {
        if(Hessian(i,i) > max)
        {
            max = Hessian(i,i);
        }
    }
    return max;
}

// L(h) = F(x)+h^t*J^t*f + h^t*J^t*J*h/2
// deltaL = h^t*(u*h-g)/2

double linearDeltaL(const VectorXd& step,const VectorXd& gradient,const double u)
{   
    double L = step.transpose()*(u*step-gradient);
    return L/2;
}
void levenMar(const VectorXd& input,VectorXd& output, VectorXd& params)
{
    int errNum = input.rows();
    int paraNum = params.rows();

    VectorXd obj = objF(input,output,params);
    MatrixXd Jac = Jacobin(input,output,params);
    MatrixXd A = Jac.transpose()*Jac;               // Hessian
    VectorXd gradient = Jac.transpose()*obj;

    double tao = 1e-3;
    long long v = 2;
    double eps1 = 1e-12,eps2 = 1e-12;

    double u = tao*maxMatrixDiagonale(A);
    bool found = gradient.norm() <= eps1;

    if(found) return;

    double last_sum = 0;
    int iterCnt = 0;

    while(iterCnt < MAX_ITER)
    {
        VectorXd obj = objF(input,output,params);

        MatrixXd Jac = Jacobin(input,output,params);
        MatrixXd A = Jac.transpose()*Jac; // Hessian
        VectorXd gradient = Jac.transpose()*obj;

        if(gradient.norm() <= eps1)
        {
            cout << " stop g(x) = 0 for a local minimizer optimizer" << endl;
            break;
        }

        VectorXd step = (A+u*MatrixXd::Identity(paraNum,paraNum)).inverse()*gradient;
        if(SHOW_DEBUG)
        {
            cout << "A: \n" << A << endl;
            cout << "step: \n" << step << endl;
        }
        

        if(step.norm() <= eps2*(params.norm()+eps2))
        {
            cout << "stop because change in x is small" << endl;
            break;
        }

        VectorXd paramsNew(params.rows());
        paramsNew = params - step;

        obj = objF(input,output,params);

        VectorXd obj_new = objF(input,output,paramsNew);

        double deltaF = Func(obj)-Func(obj_new);
        double deltaL = linearDeltaL(-1*step,gradient,u);

        double roi = deltaF/deltaL;
        
        if(roi > 0)
        {
            params = paramsNew;

            u *= max(1.0/3.0,1-pow(2*roi-1,3));
            v = 2;
        }
        else{
            u = u*v;
            v = v*2;
        }

        iterCnt++;
        if(SHOW_DEBUG)
        {
            cout << "roi :" << roi << endl;
            cout << "u= " << u << " v= " << v << endl;
            cout << "iter " << iterCnt << " times " <<endl << endl;
        }
        


    }


    cout << "LM "<< iterCnt <<endl;


}

void dogLeg(const VectorXd& input, VectorXd& output, VectorXd& params)
{
    int errNum = input.rows();
    int paraNum = params.rows();

    VectorXd obj = objF(input,output,params);
    MatrixXd Jac = Jacobin(input,output,params);
    VectorXd gradient = Jac.transpose()*obj;

    double eps1 = 1e-12,eps2 = 1e-12,eps3 = 1e-12;
    double radius = 1.0;

    bool found = obj.norm() <= eps3 || gradient.norm() <= eps1;
    if(found) return;

    double last_sum = 0;
    int iterCnt = 0;
    while(iterCnt < MAX_ITER)
    {
        VectorXd obj = objF(input,output,params);
        MatrixXd Jac = Jacobin(input,output,params);
        VectorXd gradient = Jac.transpose()*obj;

        if(gradient.norm() <= eps1)
        {
            cout << "stop F'(x) = g(x) = 0 for a global minimizer optimizer" << endl;
            break;
        }
        if(obj.norm() <= eps3)
        {
            cout << "stop f(x) = 0 for f(x) is so small" << endl;

        }
        double alpha = gradient.squaredNorm()/(Jac*gradient).squaredNorm();

        VectorXd stepest_descent = -alpha*gradient;
        VectorXd gauss_newton = (Jac.transpose()*Jac).inverse()*Jac.transpose()*obj*-1;


        double beta = 0;

        VectorXd dog_leg(params.rows());
        if(gauss_newton.norm() <= radius)
        {
            dog_leg = gauss_newton;
        }
        else if(alpha*stepest_descent.norm() >= radius)
        {
            dog_leg = (radius/stepest_descent.norm())*stepest_descent;
        }
        else{
            VectorXd a = alpha*stepest_descent;
            VectorXd b = gauss_newton;

            double c = a.transpose()*(b-a);
            beta = (sqrt(c*c+(b-a).squaredNorm()*(radius*radius-a.squaredNorm()))-c)/(b-a).squaredNorm();
            dog_leg = alpha*stepest_descent+beta*(gauss_newton-alpha*stepest_descent);

        }

        if(SHOW_DEBUG)
        {
            cout << " dog_leg: \n" << dog_leg << endl;
        }
        

        if(dog_leg.norm() <= eps2*(params.norm()+eps2))
        {
            cout << "stop because change in x is too small" << endl;
            break;
        }

        VectorXd new_params(params.rows());
        new_params = params+dog_leg;
        if(SHOW_DEBUG)
        {

            cout << "new params: \n" << new_params << endl;
        }

        obj = objF(input,output,params);

        VectorXd obj_new = objF(input,output,new_params);

        double deltaF = Func(obj)-Func(obj_new);

        double deltaL = 0;
        if(gauss_newton.norm() <= radius)
        {
            deltaL = Func(obj);
        }
        else if(alpha*stepest_descent.norm() >= radius)
        {
            deltaL = radius*(2*alpha*gradient.norm()-radius)/(2.0*alpha);

        }
        else{
            VectorXd a = alpha*stepest_descent;
            VectorXd b = gauss_newton;

            double c = a.transpose()*(b-a);

            beta = (sqrt(c*c+(b-a).squaredNorm()*(radius*radius-a.squaredNorm()))-c)/(b-a).squaredNorm();

            deltaL = alpha*(1-beta)*(1-beta)*gradient.squaredNorm()/2.0+beta*(2.0-beta)*Func(obj);
        }

        double roi = deltaF/deltaL;
        if(roi > 0)
        {
            params = new_params;
        }
        if(roi > 0.75)
        {
            radius = max(radius,3.0*dog_leg.norm());
        }
        else if(roi < 0.25)
        {
            radius = radius /2.0;
            if(radius <= eps2*(params.norm()+eps2))
            {
                cout << "trust region redius is too small" << endl;
                break;
            }
        }


        iterCnt++;
        if(SHOW_DEBUG)
        {
            cout << "roi: " << roi << " dog_leg norm: "<< dog_leg.norm() << endl;
            cout << "radius: " << radius << endl;

            cout << "iterCnt "<<iterCnt <<" times " <<endl;
        }
        
    }


    cout << "DL "<< iterCnt <<endl;

}

int getTimeofMS()
{
    struct timeval time;
    gettimeofday(&time, NULL);
    return time.tv_sec*1000 + time.tv_usec/1000;
}

int main(int argc, char* argv[])
{
    // obj = A*sin(Bx)+C*cos(Dx) -F

    int num_params = 4;
    int total_data = 100;

    VectorXd input(total_data);
    VectorXd output(total_data);
    
    cout << "test random " << (random()%1000)/1000.0 -10.0 << endl;

    double A = 5,B = 1,C = 10,D = 2;
    VectorXd tmpparam(num_params);
    tmpparam << A,B,C,D;
    for(int i = 0; i < total_data; i++)
    {
        double x = 20.0*((random()%1000)/1000.0)-10.0;
        double deltaY = 2.0 * (random()%1000)/1000.0;
        // double y = A*sin(B*x)+C*cos(D*x)+deltaY;
        
        input(i) = x;
        double y = func(input,output,tmpparam,i,false)+deltaY;
        output(i) = y;
        if(SHOW_DEBUG)
        {
            cout << x << " " << y << endl;
        }
    }

    VectorXd params_gaussNewton(num_params);
    params_gaussNewton << 1.6,1.4,6.2,1.7;

    VectorXd params_levenMar = params_gaussNewton;
    VectorXd params_dogLeg = params_gaussNewton;

    
    
    int start_time,end_time;

    // start_time = getTimeofMS();
    // sleep(1);
    // cout << " ------time " << getTimeofMS() - start_time << endl;


    start_time = getTimeofMS();
    gaussNewton(input,output,params_gaussNewton);
    cout << "gaussNewton " << params_gaussNewton << " time " << getTimeofMS() - start_time << endl;
    
    start_time = getTimeofMS();
    levenMar(input,output,params_levenMar);
    cout << "levenMar " << params_levenMar << " time " << getTimeofMS() - start_time << endl;

    start_time = getTimeofMS();
    dogLeg(input,output,params_dogLeg);
    cout << "Dog_Leg " << params_dogLeg << " time " << getTimeofMS() - start_time << endl;

}