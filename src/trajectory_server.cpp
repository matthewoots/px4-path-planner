#include <trajectory.h>

// class trajectory
// {
// public:

    void trajectory::Initialise(int order, int knotdiv, double traj_pub_rate)
    {
        _order = order;
        _knotdiv = knotdiv;
        _traj_pub_rate = traj_pub_rate;
        _fixed_cp = MatrixXd::Zero(3,1); 
        _global_cp = MatrixXd::Zero(3,1);
        _fixed_knots = VectorXd::Zero(1);
        _knot_span = (double)knotdiv * (1/traj_pub_rate);
        std::cout << KYEL << "[trajectory_server.cpp] " << "Initialise Complete" << KNRM << std::endl;
    }

    void trajectory::SetClampedPath(MatrixXd wp, 
        double max_vel, Vector3d start_pose) 
    {
        /* 
        * Uniform Distribution
        */
        MatrixXd cp_raw = MatrixXd::Zero(3,1); VectorXd time_waypoint = VectorXd::Zero(1);

        _bsp.UniformDistribution(start_pose, wp, max_vel, _knot_span, 
            &time_waypoint, &cp_raw);
        std::cout << KYEL << "[trajectory_server.cpp] " << "Uniform Distribution Complete" << KNRM << std::endl;

        /* 
        * Clamp the Bspline
        */
        // Update global control points and start and end time 
        _global_cp = _bsp.ClampBspline(_order, cp_raw);
        std::cout << KYEL << "[trajectory_server.cpp] " << "Clamping Bspline" << KNRM << std::endl;

        _fixed_cp = MatrixXd::Zero(3,_global_cp.cols());
        _fixed_cp = _global_cp;
        _start = ros::Time::now().toSec();
        _end = _start + ((_fixed_cp.cols() - (_order)) * _knot_span);
        MatrixXd _fixed_knots_tmp = _bsp.linspace(_start, _end, (double)(_global_cp.cols() - (_order-1)));
        _fixed_knots = VectorXd::Zero(_fixed_knots_tmp.cols());
        _fixed_knots = _fixed_knots_tmp.row(0);
        std::cout << KYEL << "[trajectory_server.cpp] " << "Clamping Complete" << KNRM << std::endl;
    }
    
    /** 
    * @brief Update Path with Local Control Points
    * According to Finite Time Horizon
    */
    // A requirement is that now must be an interval of the knots 
    bool trajectory::UpdatePartialPath(
        double now, double _finite_time_horizon)
    {
        // Now to _finite_time_horizon
        int start_to_end_idx = ceil(_finite_time_horizon / _knot_span);
        int start_idx;
        bool early_break = false;

        // Check if anything to evaluate in knots
        for (int i = 0; i < _fixed_knots.size(); i++)
        {
            if (now - _fixed_knots(i) == 0)
            {
                int start_idx = i;
                if (start_idx + start_to_end_idx > (_fixed_knots.size()-1))
                    int end_idx = (_fixed_knots.size()-1);
                else
                    int end_idx = i + start_to_end_idx;
                early_break = true;
                break;
            }    
        }

        // if no early break, return update path as false
        // This will give the 1st check that the agent is approximately close to agent
        if (!early_break)
            return false; 

        // Reset pos, vel, acc and time
        _pos = MatrixXd::Zero(3,1); _vel = MatrixXd::Zero(3,1); _acc = MatrixXd::Zero(3,1); 
        _time = VectorXd::Zero(1);

        // Truncate control points from global to local
        _local_cp = MatrixXd::Zero(3,start_to_end_idx + _order);
        for(int i = 0; i < _local_cp.cols(); i++)
        {
            _local_cp.col(i) = _global_cp.col(start_idx + i);
        }

        // Truncate 
        double start_local = _fixed_knots(start_idx);
        double end_local = start_local + ((double)start_to_end_idx * _knot_span);

        // Bspline Creation using Function
        _bsp.GetBspline3(_order, _local_cp, start_local, end_local, _knotdiv, 
            &_pos, &_vel, &_acc, &_time);

        std::cout << KYEL << "[trajectory_server.cpp] " << "Partial Path Update Complete" << KNRM << std::endl;
        return true;
    }

    /** 
    * @brief Update Full Path
    */
    bool trajectory::UpdateFullPath()
    {
        // Reset pos, vel, acc and time
        _pos = MatrixXd::Zero(3,1); _vel = MatrixXd::Zero(3,1); _acc = MatrixXd::Zero(3,1); 
        _time = VectorXd::Zero(1);

        // Truncate control points from global to local
        _local_cp = MatrixXd::Zero(3,_global_cp.cols());
        for(int i = 0; i < _local_cp.cols(); i++)
        {
            _local_cp.col(i) = _global_cp.col(i);
        }

        double start = _fixed_knots(0); double end = _fixed_knots(_fixed_knots.size()-1);

        // Bspline Creation using Function
        _bsp.GetBspline3(_order, _local_cp, start, end, _knotdiv, 
            &_pos, &_vel, &_acc, &_time);

        std::cout << KYEL << "[trajectory_server.cpp] " << "Full Path Update Complete" << KNRM << std::endl;
        return true;
    }

    /** 
    * @brief Get Desired State
    */
    // A requirement is that now must be an rounded to division of the knots
    bool trajectory::GetDesiredState(double now, 
        Vector3d *pos, Vector3d *vel, Vector3d *acc)
    {
        bool early_break = false;
        int idx;
        // Check the time now and compare to knots
        for(int i = 0; i < _time.size(); i++)
        {
            if(abs(_time(i) - now) < 0.001)
            {
                idx = i;
                // std::cout << KYEL << "[trajectory_server.cpp] " << "idx " << idx << KNRM << std::endl;
                early_break = true;
                break;
            }
        }
        // if no early break, return update path as false
        // This will give the 2nd check that the agent is approximately close to agent
        if (!early_break)
            return false;

        // Send back Vector3 pos vel and acc
        *pos = _pos.col(idx); *vel = _vel.col(idx); *acc = _acc.col(idx); 

        // std::cout << KYEL << "[trajectory_server.cpp] " << "Desired State Update Complete" << KNRM << std::endl;
        // std::cout << KYEL << "[trajectory_server.cpp] \n" << *pos << KNRM << std::endl;
        // std::cout << KYEL << "[trajectory_server.cpp] \n" << *vel << KNRM << std::endl;
        return true;
    }

    /** 
    * @brief Check if trajectory is completed
    */
    bool trajectory::isCompleted(double now)
    {
        if (now - _fixed_knots(_fixed_knots.size()-1) <= 0)
            return false;
        else
            return true;
    }

    /** 
    * @brief Return control point pose according to idx
    */
    bool trajectory::returnControlPointPose(int idx, Vector3d *pos)
    {
        if (idx >= 0 && idx < _fixed_cp.cols())
        {
            *pos = _fixed_cp.col(idx);
            return true;
        }
        else         
            return false;
    }


// private:

// };
