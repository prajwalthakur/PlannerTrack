//
// Created by LEE KYOUNGHEON on 2023/03/08.
//

#ifndef ELLIPSECOLLISION_OPTIMIZATION_HPP
#define ELLIPSECOLLISION_OPTIMIZATION_HPP

#include <functional>

/**
 * Optimize convex downward scalar function using gradient descent method.
 * @param x0 Initial guess of x.
 * @param f Function to optimize.
 * @return [x, f(x)] pair where f(x) is minimum.
 * @note The result may not correct (converge) if iteration count exceeds \p MAX_ITER.
 */
template <typename Func, typename T = std::invoke_result_t<Func>>
constexpr std::pair<T, T> optimize(T x0, Func &&f) {
    constexpr T AUTODIFF_GRAD_X = 1e-4,
            LEARNING_RATE = 1e-2,
            TOLERANCE = 1e-3;
    constexpr std::size_t MAX_ITER = 100;

    T x = x0;
    T fx = f(x);
    T grad_fx = (f(x + AUTODIFF_GRAD_X) - f(x)) / AUTODIFF_GRAD_X;
    for (std::size_t iter_count = 0; iter_count < MAX_ITER && abs(grad_fx) > TOLERANCE; ++iter_count){
        x -= LEARNING_RATE * grad_fx;
        fx = f(x);
        grad_fx = (f(x + AUTODIFF_GRAD_X) - f(x)) / AUTODIFF_GRAD_X;
    }
    return std::make_pair(x, fx);
}

#endif //ELLIPSECOLLISION_OPTIMIZATION_HPP
