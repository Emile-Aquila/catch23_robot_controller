//
// Created by emile on 23/09/08.
//

#include <main_arm_controller/trajectory/spline_utils.hpp>
#include <main_arm_controller/utils/robot_state.hpp>



template<class T>bool chmax(T &former, const T &b) { if (former<b) { former=b; return true; } return false; }
template<class T>bool chmin(T &former, const T &b) { if (b<former) { former=b; return true; } return false; }


void create_time_grid(std::vector<double>& T, double& tmin, double& tmax,
                      std::vector<double>& X, std::vector<double>& Y, bool is_closed_curve){
    assert(X.size()==Y.size() && X.size()>2);

    // hack for closed curves (so that it closes smoothly):
    //  - append the same grid points a few times so that the spline
    //    effectively runs through the closed curve a few times
    //  - then we only use the last loop
    //  - if periodic boundary conditions were implemented then
    //    simply setting periodic bd conditions for both x and y
    //    splines is sufficient and this hack would not be needed
    int idx_first=-1, idx_last=-1;
    if(is_closed_curve) {
        // remove last point if it is identical to the first
        if(X[0]==X.back() && Y[0]==Y.back()) {
            X.pop_back();
            Y.pop_back();
        }

        const int num_loops=3;  // number of times we go through the closed loop
        std::vector<double> Xcopy, Ycopy;
        for(int i=0; i<num_loops; i++) {
            Xcopy.insert(Xcopy.end(), X.begin(), X.end());
            Ycopy.insert(Ycopy.end(), Y.begin(), Y.end());
        }
        idx_last  = (int)Xcopy.size()-1;
        idx_first = idx_last - (int)X.size();
        X = Xcopy;
        Y = Ycopy;

        // add first point to the end (so that the curve closes)
        X.push_back(X[0]);
        Y.push_back(Y[0]);
    }

    // setup a "time variable" so that we can interpolate x and y
    // coordinates as a function of time: (X(t), Y(t))
    T.resize(X.size());
    T[0]=0.0;
    for(size_t i=1; i<T.size(); i++) {
        // time is proportional to the distance, i.e. we go at a const speed
        T[i] = T[i-1] + sqrt( pow(X[i]-X[i-1], 2.0) + pow(Y[i]-Y[i-1], 2.0) );
    }
    if(idx_first<0 || idx_last<0) {
        tmin = T[0] - 0.0;
        tmax = T.back() + 0.0;
    } else {
        tmin = T[idx_first];
        tmax = T[idx_last];
    }
}



std::vector<ArmState> path_func(const std::vector<ArmState>& waypoints, double length){
    // states -> rs, thetas
    std::vector<double> rs, thetas, phis;
    std::for_each(waypoints.begin(), waypoints.end(),[&rs, &thetas, &phis](auto& tmp){
        rs.emplace_back(tmp.r);
        thetas.emplace_back(tmp.theta);
        phis.emplace_back(tmp.phi);
    });

    // parametric spline
    tk::spline::spline_type line_type = tk::spline::cspline;
    double t_min = 0.0, t_max = 0.0;
    std::vector<double> times; // parameter
    create_time_grid(times, t_min, t_max, rs, thetas, false);

    tk::spline spline_rs, spline_thetas;
    spline_rs.set_points(times, rs, line_type);
    spline_thetas.set_points(times, thetas, line_type);


    int n = 30;
    chmax(n, (int) ceil((t_max - t_min) / length));
    std::cout << "t_range: " << t_max - t_min << std::endl;
    std::vector<ArmState> ans;
    for(int i=0; i<n; i++){
        double t = t_min + (double)i*(t_max - t_min)/((double)(n-1));
        double phi = phis[0] + (phis[phis.size()-1] - phis[0]) * (double)i / (double)(n-1);
        ans.emplace_back(spline_rs(t), spline_thetas(t), 0.0, phi);
    }
    return ans;
}
