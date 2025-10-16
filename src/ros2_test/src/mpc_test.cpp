#include <stdio.h>
#include <dlfcn.h>

#include <casadi/casadi.hpp>
#include <filesystem>
#include <chrono>

namespace fs = std::filesystem;


int main()
{   

    fs::path exePath = fs::canonical("/proc/self/exe").string(); 
    std::cout << "Executable Path: " << exePath << std::endl;

    std::cout << "---" << std::endl;
    std::cout << "Usage from CasADi C++:" << std::endl;
    std::cout << std::endl;

    std::string solver = "ALIP_MPC_Ns4_Ni4_qrqp";
    std::string libfile = "src/bxi_controller/lib/ALIP_MPC_Ns4_Ni4_qrqp.so";
    casadi::Function f = casadi::external(solver,libfile);
    std::cout << "--> External Functions Generated\n";

    // input 
    /*
     x0,# 测量值 4 x 1
        p_stance_sign, # 支撑脚 left +1 right -1  1
        m, #质量 1
        zh, #质心高度 1
        Ts, #周期 1 
        Tr, #剩余时间 1
        width,# 两腿宽度 1
        vx,
        vy,
        w,
        p_ufp_max, # 踩踏输入最大值 2 x 1
        p_ufp_min, # 踩踏输入最小值 2 x 1
        mu, # 摩擦系数 2 x 1
        k, # 斜坡斜率  2x 1
        Q_matrix, # cost函数的Q矩阵 4 x 4
    */
    double num_step = 4;
    std::vector<double> x0 = {-0.0230, 0, 0, -10};
    double p_stance_sign = 1;
    double m = 25;
    double zh = 0.67;
    double Ts = 0.3;
    double Tr = 0.29;
    double width = 0.2;
    double vx = 0;
    double vy = 0;
    double w = 0;
    std::vector<double> p_ufp_max = {0.8, 0.8};
    std::vector<double> p_ufp_min = {-0.8, 0.05};
    casadi::DM mu = casadi::DM::ones(2, 1);
    casadi::DM k = casadi::DM::zeros(2, 1);
    casadi::DM Q_matrix = casadi::DM::eye(4);
 

    std::vector<casadi::DM> arg = {x0, p_stance_sign, m, zh, Ts, Tr, width, vx, vy, w, p_ufp_max, p_ufp_min, mu, k, Q_matrix};
    std::cout<< "--> Input Arguments Generated\n";

    auto begin = std::chrono::high_resolution_clock::now();
    std::vector<casadi::DM> res;
    int iter = 1000;
    for(int i = 0; i < iter; i++){
        res = f(arg);
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto time = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
    std::cout << "Average Time: " << float(time)/float(iter) << " us" << std::endl;
    std::cout << "result (0): " << res.at(0) << std::endl;
    std::cout << "result (1): " << res.at(1) << std::endl;
    

    
}

/*
void usage_cplusplus(){
  std::cout << "---" << std::endl;
  std::cout << "Usage from CasADi C++:" << std::endl;
  std::cout << std::endl;

  // Use CasADi's "external" to load the compiled function
  Function f = external("f");

  // Use like any other CasADi function
  std::vector<double> x = {1, 2, 3, 4};
  std::vector<DM> arg = {reshape(DM(x), 2, 2), 5};
  std::vector<DM> res = f(arg);

  std::cout << "result (0): " << res.at(0) << std::endl;
  std::cout << "result (1): " << res.at(1) << std::endl;
}
  */

