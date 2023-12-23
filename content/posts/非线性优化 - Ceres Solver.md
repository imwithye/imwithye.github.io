---
title: "非线性优化 - Ceres Solver"
date: 2023-12-23
tags: ["Math"]
---

我们考虑这个优化问题：

$$
\min_{\mathbf{x}} \quad \frac{1}{2} \sum_{i} \rho_i \left(||f_i (x_{i_1}, ... ,x_{i_k})||^2\right)
$$

$$
\text{s.t.} \quad l_j \le x_j \le u_j
$$

这个问题又称为**非线性最小二乘**问题。

具体来说，我们寻找$x$的最小值，并且$x$满足了约束$l_j \le x_j \le u_j$，使得函数$\rho_i \left(||f_i (x_{i_1}, ... ,x_{i_k})||^2\right)$的和最小。我们又把$\rho_i \left(||f_i (x_{i_1}, ... ,x_{i_k})||^2\right)$称之为**残差块（ResidualBlock）**，$f_i (x_{i_1}, ... ,x_{i_k})$成为**代价函数（CostFunction）**，且依赖于$x_i$。$x_i$称为**参数块（ParameterBlock）**。同时参数块受到了$l_i$和$u_i$的约束。

特别的这里的$\rho_i$称为**损失函数（LossFunction）**，损失函数是一个标量值，用于减少外值对解的影响。

特别的，当我们的$\rho_i = 1$且上下界都为无穷大时，我们的问题简化为:

$$
\min_{\mathbf{x}} \quad \frac{1}{2} \sum_{i} \left(||f_i (x_{i_1}, ... ,x_{i_k})||^2\right)
$$

## 一个最简单的例子

我们考虑优化一个函数：

$$
\frac{1}{2}(10-x)^2
$$

我们用`Ceres Solver`来求解这个问题。很显然，这里的代价函数是$f(x) = 10 - x$，$\rho$为1，且$x$在实数域上没有约束。所以我们定义我们的代价函数

```c++
struct CostFunctor {
    template <typename T>
    bool operator()(const T* const x, T* residual) const {
        residual[0] = 10.0 - x[0];
        return true;
    }
}
```

现在我们可以解决这个优化问题：

```c++
int main(int argc, char** argv) {
    // x的初始值为5.0
    double initial_x = 5.0;
    double x = initial_x;

    // 定义待解决的问题，并初始化代价函数，同时利用自动微分求解代价函数的梯度
    Problem problem;
    CostFunction* cost_function = new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
    problem.AddResidualBlock(cost_function, nullptr, &x);

    // 求解问题
    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    // 求解结束
    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x << " -> " << x << "\n";

    return 0;
}
```

我们来执行一下这段代码片段，经过3次迭代以后，ceres求解出$x$为10。

这里有几个函数细节需要解释。`AutoDiffCostFunction<CostFunctor, 1, 1>`是自动微分模块，ceres根据代价函数的定义自动求解微分。在模版定义中，第一个参数为代价函数，第二个参数是残差块的维度（对于我们上面的例子来说，残差块是个标量），第三个函数是参数块的纬度（同样参数块也是标量）。

自动微分模块只是ceres的其中一种微分模块，根据不同的需要ceres提供了多种不同的微分模块，例如数值微分模块。注意自动微分和数值微分在定义上只想差了一个模版类，但是自动微分的效率要远远好于数值微分。当我们的残差函数能被良好定义时，推荐使用自动微分模块。当然，ceres提供了接口提供自定义微分函数，用户可以自行输出函数对应的雅可比矩阵。

