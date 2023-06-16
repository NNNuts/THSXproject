#ifndef _KINEMATICS_
#define _KINEMATICS_
#include <iostream>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;


#define Rpy Vector3d

//关节数值
class Joints
{
public:
    Joints(double jointsNum[6])
    {
        joint0 = joints[0] = jointsNum[0];
        joint1 = joints[1] = jointsNum[1];
        joint2 = joints[2] = jointsNum[2];
        joint3 = joints[3] = jointsNum[3];
        joint4 = joints[4] = jointsNum[4];
        joint5 = joints[5] = jointsNum[5];
    }


    double joint0;
    double joint1;
    double joint2;
    double joint3;
    double joint4;
    double joint5;

    double joints[6];
};

class TrainComputer
{
public:
    
    // double d[6] = { 101.5, 0, 0, 79.2, 079.2, 41.7 };//第0个不用
    // double a[6] = { 0, -173, -173, 0, 0, 0 };//a有0，没有6
    // double alpha[6] = { 90, 0, 0, 90, -90, 0 };//alpha有0，没有6

    double d[6]          = { 185.2,           0,           0, 163.9, 115.7, 102.7 };//第0个不用
    double a[6]          = {          0, -612.7, -571.6,         0,             0,         0 };//a有0，没有6
    double alpha[6] = { 90, 0, 0, 90, -90, 0 };//alpha有0，没有6
    double theta[8][6];//八组解，每组解六个角，第0个都不用
    double theta_now[6];//记录理论当前机械臂位置

    //传递矩阵
    Matrix4d Trans(int i, double theta_input)
    {
        Matrix4d T;
        T(0, 0) = cos(theta_input);
        T(0, 1) = -sin(theta_input) * cos(alpha[i] / 180 * EIGEN_PI);
        T(0, 2) = sin(theta_input) * sin(alpha[i] / 180 * EIGEN_PI);
        T(0, 3) = a[i] * cos(theta_input);
        T(1, 0) = sin(theta_input);
        T(1, 1) = cos(theta_input) * cos(alpha[i] / 180 * EIGEN_PI);
        T(1, 2) = -cos(theta_input) * sin(alpha[i] / 180 * EIGEN_PI);
        T(1, 3) = a[i] * sin(theta_input);
        T(2, 0) = 0;
        T(2, 1) = sin(alpha[i] / 180 * EIGEN_PI);
        T(2, 2) = cos(alpha[i] / 180 * EIGEN_PI);
        T(2, 3) = d[i];
        T(3, 0) = 0;
        T(3, 1) = 0;
        T(3, 2) = 0;
        T(3, 3) = 1;
        return T;
    }


    //正运动学
    //输入：1x6 数组，为0-5号关节数值（弧度）
    //输出：Matrix4d 4x4姿态矩阵
    //---------------
    Matrix4d kinematics(double theta_input[6])
    {
        /*cout << " theta_input1:" << theta_input[0] / EIGEN_PI * 180;
        cout << " theta_input2:" << theta_input[1] / EIGEN_PI * 180;
        cout << " theta_input3:" << theta_input[2] / EIGEN_PI * 180;
        cout << " theta_input4:" << theta_input[3] / EIGEN_PI * 180;
        cout << " theta_input5:" << theta_input[4] / EIGEN_PI * 180;
        cout << " theta_input6:" << theta_input[5] / EIGEN_PI * 180 << endl;*/
        Matrix4d T[6];
        for (int i = 0; i < 6; i++)
        {
            T[i] = Trans(i, theta_input[i]);
            // T[i](0, 0) = cos(theta_input[i]);
            // T[i](0, 1) = -sin(theta_input[i]) * cos(alpha[i] / 180 * EIGEN_PI);
            // T[i](0, 2) = sin(theta_input[i]) * sin(alpha[i] / 180 * EIGEN_PI);
            // T[i](0, 3) = a[i] * cos(theta_input[i]);
            // T[i](1, 0) = sin(theta_input[i]);
            // T[i](1, 1) = cos(theta_input[i]) * cos(alpha[i] / 180 * EIGEN_PI);
            // T[i](1, 2) = -cos(theta_input[i]) * sin(alpha[i] / 180 * EIGEN_PI);
            // T[i](1, 3) = a[i] * sin(theta_input[i]);
            // T[i](2, 0) = 0;
            // T[i](2, 1) = sin(alpha[i] / 180 * EIGEN_PI);
            // T[i](2, 2) = cos(alpha[i] / 180 * EIGEN_PI);
            // T[i](2, 3) = d[i];
            // T[i](3, 0) = 0;
            // T[i](3, 1) = 0;
            // T[i](3, 2) = 0;
            // T[i](3, 3) = 1;
        }
        Eigen::Matrix4d T06 = T[0] * T[1] * T[2] * T[3] * T[4] * T[5];

        return T06;
    }

    //逆解 
    /*
    T={Nx,Ox,Ax,Px,
       Ny,Oy,Ay,Py,
       Nz,Oz,Az,Pz,
        0, 0, 0, 0}
    */
    //逆运动学
    //输入：1x12 数组，0-2 为 x y z 数值（mm），3-11 为 3x3 的姿态矩阵数值
    //输出：1x6 数组，为0-5号关节数值（弧度）
    //特别：无
    int invest(Matrix4d T, double* ret)
    {
        double x, y, z, Nx, Ny, Nz, Ox, Oy, Oz, Ax, Ay, Az;
        double m, n;
        double S2, C2;
        x = T(0, 3); 
        y = T(1, 3);
        z = T(2, 3);
        //Eigen::Matrix3d rm;
        Nx = T(0, 0);
        Ox = T(0, 1);
        Ax = T(0, 2);
        Ny = T(1, 0);
        Oy = T(1, 1);
        Ay = T(1, 2);
        Nz = T(2, 0);
        Oz = T(2, 1);
        Az = T(2, 2);
        //2.求解

        //theta 0
        m = d[5] * Ay - y;
        n = d[5] * Ax - x;
        //C = d[4];
        //第一个解，赋给一到四组
        theta[0][0] = atan2(m, n) - atan2(d[3], sqrt(m * m + n * n - d[3] * d[3]));
        theta[1][0] = theta[0][0];
        theta[2][0] = theta[0][0];
        theta[3][0] = theta[0][0];
        //第二个解，赋给五到八组
        theta[4][0] = atan2(m, n) - atan2(d[3], -sqrt(m * m + n * n - d[3] * d[3]));
        theta[5][0] = theta[4][0];
        theta[6][0] = theta[4][0];
        theta[7][0] = theta[4][0];

        //theta 4
        //由theta[1][1]产生的第一个解，赋给一到二组
        //A = sin(theta[1][1]) * rm[0][2] - cos(theta[1][1]) * rm[1][2];
        theta[0][4] = acos(Ax * sin(theta[0][0]) - Ay * cos(theta[0][0]));
        theta[1][4] = theta[0][4];
        //由theta[1][1]产生的第二个解，赋给三到四组
        theta[2][4] = -acos(Ax * sin(theta[0][0]) - Ay * cos(theta[0][0]));
        theta[3][4] = theta[2][4];
        //由theta[5][1]产生的第一个解，赋给五到六组
        //A = sin(theta[5][1]) * rm[0][2] - cos(theta[5][1]) * rm[1][2];
        theta[4][4] = acos(Ax * sin(theta[4][0]) - Ay * cos(theta[4][0]));
        theta[5][4] = theta[4][4];
        //由theta[5][1]产生的第二个解，赋给七到八组
        theta[6][4] = -acos(Ax * sin(theta[4][0]) - Ay * cos(theta[4][0]));
        theta[7][4] = theta[6][4];

        //theta 5
        for (int i = 0; i < 8; i++)
        {
            //cout<<"Nx: "<<Nx<<" Ny: "<<Ny<<" theta1: "<<theta[i][1]<<endl;
            //cout<<"Ox: "<<Ox<<" Oy: "<<Oy<<endl;
            m = Nx * sin(theta[i][0]) - Ny * cos(theta[i][0]);
            n = Ox * sin(theta[i][0]) - Oy * cos(theta[i][0]);
            //cout<<"m: "<<m<<" n: "<<n<<endl;
            theta[i][5] = atan2(-n / sin(theta[i][4]), m / sin(theta[i][4]));
        }

        //theta 2
        for (int i = 0; i < 8; i = i + 2)
        {
            m = d[4] * (sin(theta[i][5]) * (Nx * cos(theta[i][0]) + Ny * sin(theta[i][0])) + cos(theta[i][5]) * (Ox * cos(theta[i][0]) + Oy * sin(theta[i][0])))
                - d[5] * (Ax * cos(theta[i][0]) + Ay * sin(theta[i][0])) + x * cos(theta[i][0]) + y * sin(theta[i][0]);
            n = z - d[0] - Az * d[5] + d[4] * (Oz * cos(theta[i][5]) + Nz * sin(theta[i][5]));

            theta[i][2] = acos((m * m + n * n - a[1] * a[1] - a[2] * a[2]) / (2 * a[1] * a[2]));
            theta[i + 1][2] = -acos((m * m + n * n - a[1] * a[1] - a[2] * a[2]) / (2 * a[1] * a[2]));
        }
        //theta 1 、 theta 3
        for (int i = 0; i < 8; i++)
        {
            m = d[4] * (sin(theta[i][5]) * (Nx * cos(theta[i][0]) + Ny * sin(theta[i][0])) + cos(theta[i][5]) * (Ox * cos(theta[i][0]) + Oy * sin(theta[i][0])))
                - d[5] * (Ax * cos(theta[i][0]) + Ay * sin(theta[i][0])) + x * cos(theta[i][0]) + y * sin(theta[i][0]);
            n = z - d[0] - Az * d[5] + d[4] * (Oz * cos(theta[i][5]) + Nz * sin(theta[i][5]));

            S2 = ((a[2] * cos(theta[i][2]) + a[1]) * n - (a[2] * sin(theta[i][2]) * m)) / (a[1] * a[1] + a[2] * a[2] + 2 * a[1] * a[2] * cos(theta[i][2]));
            C2 = (m + a[2] * sin(theta[i][2]) * S2) / (a[2] * cos(theta[i][2]) + a[1]);
            //cout<<C2<<endl;
            theta[i][1] = atan2(S2, C2);

            theta[i][3] = atan2(-sin(theta[i][5]) * (Nx * cos(theta[i][0]) + Ny * sin(theta[i][0])) - cos(theta[i][5]) * (Ox * cos(theta[i][0]) + Oy * sin(theta[i][0])),
                               (Oz * cos(theta[i][5]) + Nz * sin(theta[i][5]))) - theta[i][1] - theta[i][2];
        }
        for (int a = 0; a < 8; a++)
        {
            for (int b = 0; b < 6; b++)
            {
                *ret = theta[a][b];
                ret++;
            }
        }
        return 0;
    }


    template<typename _Matrix_Type_> _Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon()) 
    {  
        Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);  
        double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);  
        return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint(); 
    }  


    //数值解逆运动学
    //输入：1x12 数组，0-2 为 x y z 数值（mm），3-11 为 3x3 的姿态矩阵数值
    //输出：1x6 数组，为0-5号关节数值（弧度）
    //特别：无
    int iterate_ikine(double init_point[6] ,  Matrix4d target,  double *result, double efs=pow(10, -6),  double i_max=100)
    {
        // exit(0);
        // 计数及标签
        double deltaQ = 1;
        double temp_count = 0;
        VectorXd Init(6);
        Init(0) = init_point[0];
        Init(1) = init_point[1];
        Init(2) = init_point[2];
        Init(3) = init_point[3];
        Init(4) = init_point[4];
        Init(5) = init_point[5];
        //Init << init_point[0], init_point[1], init_point[2], init_point[3], init_point[4], init_point[5];

        //exit(0);
        //迭代循环求解
        while (deltaQ > efs)
        {
            double path[6] = {Init(0),Init(1),Init(2),Init(3),Init(4),Init(5)};
            Matrix4d An = kinematics(path);

            // 计算末端误差
            VectorXd dA(6);
            dA(0) = target(0, 3) - An(0, 3);
            dA(1) = target(1, 3) - An(1, 3);
            dA(2) = target(2, 3) - An(2, 3);
            Vector3d A1(An(0,0), An(1,0),An(2,0));
            Vector3d A2(An(0,1), An(1,1),An(2,1));
            Vector3d A3(An(0,2), An(1,2),An(2,2));
            Vector3d T1(target(0,0), target(1,0),target(2,0));
            Vector3d T2(target(0,1), target(1,1),target(2,1));
            Vector3d T3(target(0,2), target(1,2),target(2,2));

            Vector3d d = 0.5 * (A1.cross(T1) + A2.cross(T2) + A3.cross(T3));
            dA(3) = d(0);
            dA(4) = d(1);
            dA(5) = d(2);
            //cout<<dA<<endl<<endl;


            // 计算雅克比矩阵
            // U = np.eye(4)
            Matrix4d U = MatrixXd::Identity(4,4);
            //Jn = np.zeros([6, self.n]);
            MatrixXd Jn = MatrixXd::Zero(6,6);
            for(int i = 5; i >= 0; i--)
            {
                //i = self.n - i - 1
                U = Trans(i, Init(i)) * U;
                Jn(0, i) = -U(0, 0) * U(1, 3) + U(1, 0) * U(0, 3);
                Jn(1, i) = -U(0, 1) * U(1, 3) + U(1, 1) * U(0, 3);
                Jn(2, i) = -U(0, 2) * U(1, 3) + U(1, 2) * U(0, 3);
                Jn(3, i) = U(2, 0);
                Jn(4, i) = U(2, 1);
                Jn(5, i) = U(2, 2);

                // Jn[3:6, i] = U[2, 0:3]
            }
            // cout<<Jn<<endl<<endl;

            Matrix3d R = An.block<3,3>(0,0);
            MatrixXd J_R = MatrixXd::Zero(6,6);
            J_R.block<3,3>(0,0) = R;
            J_R.block<3,3>(3,3) = R;
            // J_R = np.zeros([6, 6])
            // J_R[0:3, 0:3] = R
            // J_R[3:6, 3:6] = R

            // cout<<J_R<<endl<<endl;

            MatrixXd J0 = J_R * Jn;
            //求取关节角关节角度偏差值
            // dq = np.dot(np.linalg.pinv(J0), dA)
            // VectorXd dq = pinv_eigen_based(J0) * dA;
            VectorXd dq = pseudoInverse(J0) * dA;

            //cout<<dq<<endl<<endl;

            Init = Init + dq;
            //cout<<Init(0)<<" "<<Init(1)<<" "<<Init(2)<<" "<<Init(3)<<" "<<Init(4)<<" "<<Init(5)<<endl;
            deltaQ = dq.lpNorm<2>();
            // deltaQ = np.linalg.norm(dq)
            // cout<<deltaQ<<endl<<endl;

            temp_count = temp_count + 1;
            if (temp_count > i_max)
            {
                printf("Solution wouldn't converge\r\n");
                *result = 10000;result++;
                *result = 10000;result++;
                *result = 10000;result++;
                *result = 10000;result++;
                *result = 10000;result++;
                *result = 10000;result++;
                return 1;
                // *result = Init(1);result++;
                // *result = Init(2);result++;
                // *result = Init(3);result++;
                // *result = Init(4);result++;
                // *result = Init(5);result++;
                // *result = init_point[0];result++;
                // *result = init_point[1];result++;
                // *result = init_point[2];result++;
                // *result = init_point[3];result++;
                // *result = init_point[4];result++;
                // *result = init_point[5];result++;
                break;
            }
        }
        for(int i = 0;i<6;i++)
        {
            while(Init(i)>EIGEN_PI)
            {
                Init(i) -= EIGEN_PI * 2;
            }
            while(Init(i)<-EIGEN_PI)
            {
                Init(i) += EIGEN_PI * 2;
            }
            *result = Init(i);result++;
        }
        //cout<<Init(0)<<Init(1)<<Init(2)<<Init(3)<<Init(4)<<Init(5)<<endl;
        // q_tmp = q_r - self.theta
        // q = self.qq_choose(Init)
        return 1;
    }

    //计算一个新姿态与原姿态的差异（平方和）
    //输入：1x6 数组，为0-5号关节数值（弧度）
    //输出：一个double数，为新旧姿态差异之平方和（弧度）
    //特别：无
    double Dis_Computer(double T[6])
    {
        double Dis = 0;
        double num = 0;
        double quanzhi[6] = { 1,1,1,1,1,1 };
        for (int i = 0; i < 6; i++)
        {
            num = fabs(theta_now[i] - T[i]);//新旧位姿误差值

            // cout<<theta_now[i]<< " " <<T[i]<<endl;
            // cout<<num<<endl;
            if (num > 6)
            {
                // cout << "跳跃 " << num << endl;
                if (T[i] > 0)
                {
                    num = ((EIGEN_PI - T[i]) + (EIGEN_PI + theta_now[i]));
                }
                else
                {
                    num = ((EIGEN_PI + T[i]) + (EIGEN_PI - theta_now[i]));
                }
            }
            // cout<<theta_now[i]<< " " <<T[i]<<endl;
            // cout<<num<<endl;
            Dis += num * num * quanzhi[i];
        }

        return Dis;
    }

    //计算一个新姿态与原姿态的差异（平方和）
    //输入：T[6] double 为0-5号新关节数值（弧度）
    //      Old[6] double 为0-5号旧关节数值（弧度）
    //输出：一个double数，为新旧姿态差异之平方和（弧度）
    //特别：无
    double Dis_ComputerWithPose(double T[6], double Old[6])
    {
        double Dis = 0;
        double num = 0;
        double quanzhi[6] = { 1,1,1,1,1,1 };
        for (int i = 0; i < 6; i++)
        {
            //cout << "old" << i << Old[i] << "T" << i << T[i] << endl;
            num = fabs(Old[i] - T[i]);//新旧位姿误差值
            //cout<<num<<endl;
            if (num > 6)
            {
                //cout << "跳跃 " << num << endl;
                if (T[i] > 0)
                {
                    num = ((EIGEN_PI - T[i]) + (EIGEN_PI + theta_now[i]));
                }
                else
                {
                    num = ((EIGEN_PI + T[i]) + (EIGEN_PI - theta_now[i]));
                }
            }
            Dis += num * num * quanzhi[i];
        }

        return Dis;
    }

    //计算各个新姿态与给定姿态的差异，并选取最小的一组
    //输入：8x6 数组，为八组 0-5号关节数值（弧度）； 
    //      Old[6], 旧关节位置数值（弧度）
    //      一个整形， 为需要计算的组数
    //      一个double 限制最大变动角度
    //输出：1x6 数组，为0-5号关节数值（弧度）
    //特别：若所有姿态都与当前姿态相差大于预定值，则第一位返回10000，代表没有合适解
    void Dis_JudgeWithPos(double T[8][6], double Old[6], int num, double Max_dis, double* Res)
    {
        double Dis[8];
        for (int i = 0; i < num; i++)
        {
            Dis[i] = Dis_ComputerWithPose(T[i], Old);
        }
        double res[6];
        res[0] = 10000;
        for (int i = 0; i < num; i++)
        {
            if (Dis[i] < Max_dis)
            {
                Max_dis = Dis[i];
                res[0] = T[i][0];
                res[1] = T[i][1];
                res[2] = T[i][2];
                res[3] = T[i][3];
                res[4] = T[i][4];
                res[5] = T[i][5];
            }
        }
        *Res = res[0]; Res++;
        *Res = res[1]; Res++;
        *Res = res[2]; Res++;
        *Res = res[3]; Res++;
        *Res = res[4]; Res++;
        *Res = res[5]; Res++;
    }
    
    //计算各个新姿态与原姿态的差异，并选取最小的一组
    //输入：8x6 数组，为八组 0-5号关节数值（弧度）； 
    //      一个整形， 为需要计算的组数
    //      一个double 限制最大变动角度
    //输出：1x6 数组，为0-5号关节数值（弧度）
    //特别：若所有姿态都与当前姿态相差大于预定值，则第一位返回10000，代表没有合适解
    void Dis_Judge(double T[8][6], int num,  double Max_dis, double* Res)
    {
        double Dis[8];
        for (int i = 0; i < num; i++)
        {
            // cout << "T"<< i<< " " <<T[i][0] << T[i][1] << T[i][2] << T[i][3] << T[i][4] << T[i][5] << endl;
            Dis[i] = Dis_Computer(T[i]);
        }
        /*cout << endl;*/
        double res[6] = {0};
        res[0] = 10000;
        for (int i = 0; i < num; i++)
        {
            // cout<<"Dis "<< i<< " "<<Dis[i]<<endl;
            if (Dis[i] < Max_dis)
            {
                // cout<<i<<endl;
                // cout << "T"<< i<< " " <<T[i][0] << T[i][1] << T[i][2] << T[i][3] << T[i][4] << T[i][5] << endl;
                res[0] = T[i][0];
                res[1] = T[i][1];
                res[2] = T[i][2];
                res[3] = T[i][3];
                res[4] = T[i][4];
                res[5] = T[i][5];
                Max_dis = Dis[i] ;
            }
        }
        if(res[0] > 100)
        {
            cout<<"num "<<num<<endl<<"Dis ";
            for(int i = 0; i < num; i++)
            {
                cout<<Dis[i]<< " ";
            }
            cout<<endl;
        }
        // cout << "res "<< " " <<res[0] << res[1] << res[2]<<res[3] << res[4]<< res[5] << endl;
        *Res = res[0]; Res++;
        *Res = res[1]; Res++;
        *Res = res[2]; Res++;
        *Res = res[3]; Res++;
        *Res = res[4]; Res++;
        *Res = res[5]; Res++;
    }

    //根据目标点末端位姿逆解，并选择合适姿态（最小差异原则）
    //输入：T[5]    1x5 数组，0-2为x y z 坐标，3为绕世界坐标系z轴旋转，4为绕末端坐标系x轴旋转
    //      Old[6] double 旧机械臂位姿
    //      Max_err 允许单步最大变化幅度
    //输出：Res 1x6 数组，为0-5号关节数值（弧度）
    //特别：若所有姿态都与当前姿态相差大于预定值，则第一位返回10000，代表没有合适解
    void MoveJ_InvestAndJudge(double T[6], double Old[6], double* Res, double Max_err = 0.5)
    {
        Matrix4d input;
        Rpy rpy(T[3], T[4], T[5]);
        input.block<3, 3>(0, 0) = Rpy2Matrix(rpy);
        input(0, 3) = T[0];
        input(1, 3) = T[1];
        input(2, 3) = T[2];

        double TT[8][6];
        double theta[8][6];

        invest(input, theta[0]);

        int t = 0;
        for (int i = 0; i < 8; i++)
        {
            TT[t][0] = theta[i][0];
            if (theta[i][0] < -EIGEN_PI)
                TT[t][0] = theta[i][0] + 2 * EIGEN_PI;
            if (isnan(TT[t][0]))
                continue;

            TT[t][1] = theta[i][1];
            if (theta[i][1] > EIGEN_PI / 2)
                TT[t][1] = theta[i][1] - 2 * EIGEN_PI;
            if (isnan(TT[t][1]))
                continue;

            TT[t][2] = theta[i][2];
            if (isnan(TT[t][2]))
                continue;

            TT[t][3] = theta[i][3];
            if (theta[i][3] > EIGEN_PI / 2)
                TT[t][3] = -(2 * EIGEN_PI - theta[i][3]);
            if (theta[i][3] < -EIGEN_PI / 4 * 3)
                TT[t][3] = theta[i][3] + 2 * EIGEN_PI;
            if (isnan(TT[t][3]))
                continue;

            TT[t][4] = theta[i][4];
            if (isnan(TT[t][4]))
                continue;

            TT[t][5] = theta[i][5];
            if (isnan(TT[t][5]))
                continue;

            t++;
        }
        double res[6];

        Dis_JudgeWithPos(TT, Old, t, Max_err, res);

        *Res = res[0]; Res++;
        *Res = res[1]; Res++;
        *Res = res[2]; Res++;
        *Res = res[3]; Res++;
        *Res = res[4]; Res++;
        *Res = res[5]; Res++;
    }

    //根据rpy求旋转矩阵（3x3）
    //输入：rpy    Rpy 结构体 rx ry rz 数值（弧度）
    //输出：3x3 旋转矩阵
    //特别：无
    Matrix3d Rpy2Matrix(Rpy rpy)
    {
        Matrix3d chk_mat = (AngleAxisd(rpy[2], Vector3d::UnitZ())
            * AngleAxisd(rpy[1], Vector3d::UnitY())
            * AngleAxisd(rpy[0], Vector3d::UnitX())).toRotationMatrix();
            
        return chk_mat;
    }

    //根据旋转矩阵（3x3）求rpy
    //输入：3x3 旋转矩阵
    //输出：rpy    Rpy 结构体 rx ry rz 数值（弧度）
    //特别：无
    Rpy Matrix2Rpy(Matrix3d matrix)
    {
        Rpy rpy = matrix.eulerAngles(2, 1, 0);
        
        Rpy eulerAngle(rpy(2), rpy(1), rpy(0));
        return eulerAngle;
    }

    //根据rpy求四元素
    //输入：rpy    Rpy 结构体 rx ry rz 数值（弧度）
    //输出：q Quaterniond结构体 
    //特别：无
    Quaterniond Rpy2Quaternion(Rpy rpy)
    {
        double cosRoll = cos(rpy(0) * 0.5);
        double sinRoll = sin(rpy(0) * 0.5);

        double cosPitch = cos(rpy(1) * 0.5);
        double sinPitch = sin(rpy(1) * 0.5);

        double cosHeading = cos(rpy(2) * 0.5);
        double sinHeading = sin(rpy(2) * 0.5);
        
        double w = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
        double x = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
        double y = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
        double z = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;
        Quaterniond q(w,x,y,z);

        return q;
    }

    //根据四元素求rpy
    //输入：q Quaterniond结构体
    //输出：rpy    Rpy 结构体 rx ry rz 数值（弧度）
    //特别：无
    Rpy Quaternion2Rpy(Quaterniond q)
    {
        Rpy rpy;
        double q0 = q.w();
        double q1 = q.x();
        double q2 = q.y();
        double q3 = q.z();
        rpy(2) = atan2(2 * (q3 * q0 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
        rpy(1) = asin(2 * (q0 * q2 - q3 * q1));
        rpy(0) = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q2 * q2 + q1 * q1));

        return rpy;
    }

    //根据四元素求旋转矩阵
    //输入：q Quaterniond结构体
    //输出：3x3旋转矩阵
    //特别：无
    Matrix3d Quaternion2Matrix(Quaterniond q)
    {
        Rpy rpy = Quaternion2Rpy(q);

        return Rpy2Matrix(rpy);
    }

    //根据旋转矩阵求四元素
    //输入：3x3旋转矩阵
    //输出：q Quaterniond结构体
    //特别：无
    Quaterniond Matrix2Quaternion(Matrix3d matrix)
    {
        Rpy rpy = Matrix2Rpy(matrix);

        return Rpy2Quaternion(rpy);
    }

    //四元素的球面线性插补
    //输入：starting Quaterniond结构体 开始姿态
    //      ending   Quaterniond结构体 结束姿态
    //      time     float  间隔时间（0-1之间占比）
    //输出：Quaterniond结构体 当前姿态
    //特别：无
    Quaterniond Slerp(Quaterniond starting, Quaterniond ending, float time)
    {
        
        float cosa = starting.w() * ending.w() + starting.x() * ending.x() + starting.y() * ending.y() + starting.z() * ending.z();

        if (cosa < 0.0f)
        {
            Quaterniond ending(-ending.w(), -ending.x(), -ending.y(), -ending.z());
            cosa = -cosa;
        }
        double k0, k1;
        double halfTheta = (double) acos((double) cosa);
        double sinHalfTheta = (double) sqrt((double) (1.0F - cosa * cosa));
        // double ratioA;
        // double ratioB;
        if ((double) abs(sinHalfTheta) > 0.001) 
        {
            double oneOverSinHalfTheta = 1.0F / sinHalfTheta;
            k0 = (double) sin((double) ((1.0F - time) * halfTheta)) * oneOverSinHalfTheta;
            k1 = (double) sin((double) (time * halfTheta)) * oneOverSinHalfTheta;
        } 
        else 
        {
            k0 = 1.0F - time;
            k1 = time;
        }
        

        // if (cosa > 0.9995f)
        // {
        //     /*cout << "使用Nlerp" << endl;*/
        //     k0 = 1.0f - time;
        //     k1 = time;
        // }
        // else
        // {
        //     float sina = sqrt(1.0f - cosa * cosa);
        //     float a = atan2(sina, cosa);
        //     k0 = sin((1.0f - time) * a) / sina;
        //     k1 = sin(time * a) / sina;
        // }
        Quaterniond result(starting.w() * k0 + ending.w() * k1, starting.x() * k0 + ending.x() * k1, starting.y() * k0 + ending.y() * k1, starting.z() * k0 + ending.z() * k1);

        return result;
    }

    

    //根据目标点末端位姿逆解，并选择合适姿态（最小差异原则）
    //输入：T[6]    1x6 数组
    //      Max_err 允许单步最大变化幅度
    //输出：1x6 数组，为0-5号关节数值（弧度）
    //特别：无
    void InvestAndJudge(double T[6], double* Res, double Max_err = 0.5)
    {
        Matrix4d input;
        Rpy rpy(T[3], T[4], T[5]);
        input.block<3, 3>(0, 0) = Rpy2Matrix(rpy);
        input(0, 3) = T[0];
        input(1, 3) = T[1];
        input(2, 3) = T[2];
        input(3, 0) = 0;
        input(3, 1) = 0;
        input(3, 2) = 0;
        input(3, 3) = 1;

        // cout<<"input"<<endl<<input<<endl;

        double TT[8][6];
        double theta[8][6];

        invest(input, theta[0]);
        
        int t = 0;
        for (int i = 0; i < 8; i++)
        {
            TT[t][0] = theta[i][0];
            if (theta[i][0] < -EIGEN_PI)
                TT[t][0] = theta[i][0] + 2 * EIGEN_PI;
            if (isnan(TT[t][0]))
                continue;

            TT[t][1] = theta[i][1];
            if (theta[i][1] > EIGEN_PI / 2)
                TT[t][1] = theta[i][1] - 2 * EIGEN_PI;
            if (isnan(TT[t][1]))
                continue;

            TT[t][2] = theta[i][2];
            if (isnan(TT[t][2]))
                continue;

            TT[t][3] = theta[i][3];
            if (theta[i][3] > EIGEN_PI / 2)
                TT[t][3] = -(2 * EIGEN_PI - theta[i][3]);
            if (theta[i][3] < -EIGEN_PI * 2)
                TT[t][3] = theta[i][3] + 2 * EIGEN_PI;
            if (isnan(TT[t][3]))
                continue;

            TT[t][4] = theta[i][4];
            if (isnan(TT[t][4]))
                continue;

            TT[t][5] = theta[i][5];
            if (isnan(TT[t][5]))
                continue;
            
            t++;
        }
        // cout<<"共有"<<t<<"个解"<<endl;
        double res[6];

        Dis_Judge(TT, t, Max_err, res);
        if(res[0] == 10000)
        {
            // iterate_ikine(theta_now,input,res);
        }
        
        *Res = res[0]; Res++;
        *Res = res[1]; Res++;
        *Res = res[2]; Res++;
        *Res = res[3]; Res++;
        *Res = res[4]; Res++;
        *Res = res[5]; Res++;
    }

    //获取该机械臂末端姿态rpy
    //输入：无
    //输出：Rpy
    //特别：无
    Rpy GetNowRpy(double joint[6])
    {
        Matrix4d matrix = kinematics(joint);
        return Matrix2Rpy(matrix.block<3, 3>(0, 0));
    }
};


#endif