#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>

/// \brief simple point in a trajectory
typedef struct TrajectoryPoint {
    double x;
    double y;
    double z;

    TrajectoryPoint() { x = 0.0; y = 0.0; z = 0.0; }
    TrajectoryPoint(double xx, double yy) : x(xx), y(yy) { z = 0.0; }
    TrajectoryPoint(double xx, double yy, double zz) : x(xx), y(yy), z(zz) { }
    TrajectoryPoint(double* p) { x = p[0]; y = p[1]; z = p[2]; }

    TrajectoryPoint& operator=(TrajectoryPoint const& copy)
    {
        x = copy.x;
        y = copy.y;
        z = copy.z;
        return *this;
    }

} Tpoint;


/// \class Trajectory
/// \brief A list of point for the robot agent to follow
class Trajectory {

public:


     Trajectory();

    ~Trajectory() { path_.clear();}


    void addPointEnd(Tpoint p);

    void addVelocities(double v_, double w_);

    void addPointBegin(Tpoint p);

    int length() {return path_.size(); }

    void resample(const int new_length);

    void reset() { path_.clear(); v.clear(); w.clear();}

    std::vector<Tpoint> getPath() {return path_;}





    std::vector<double> v;
    std::vector<double> w;

    Tpoint getNextPoint(Tpoint p);

private:

    std::vector<Tpoint> path_;
    std::vector<Tpoint> drive_path_;


};

#endif // TRAJECTORY_H
