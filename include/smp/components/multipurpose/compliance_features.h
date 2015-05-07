/**
* Copyright 2014 Social Robotics Lab, University of Freiburg
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*    # Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*    # Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*    # Neither the name of the University of Freiburg nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* \author Billy Okal <okal@cs.uni-freiburg.de>
*/


#ifndef UTILITIES_H
#define UTILITIES_H

#include <boost/foreach.hpp>
#include <boost/array.hpp>

#include <vector>
#include <set>
#include <algorithm>

typedef boost::array<double, 2> Array2D;
typedef boost::array<double, 4> Array4D;
typedef boost::array<int, 2> Array2I;


/// -----------------------------------------------------------
/// \brief Euclidean distance bwtween two 2D arrays
/// -----------------------------------------------------------
inline double edist(Array2D v1, Array2D v2)
{
    return sqrt((v1[0]-v2[0])*(v1[0]-v2[0]) + (v1[1]-v2[1])*(v1[1]-v2[1]));
}

inline double edist(Array4D v1, Array4D v2)
{
    return sqrt( (v1[0]-v2[0])*(v1[0]-v2[0]) + (v1[1]-v2[1])*(v1[1]-v2[1]) );
}


/// -----------------------------------------------------------
/// \brief Compute the cost of an action given features and
/// weights
/// -----------------------------------------------------------
inline double computeActionCost(std::vector<double> features, std::vector<double> weights)
{
    double cost = 0.0;
    for (int i = 0; i < 3; i++)
        cost += features[i] * weights[i];

    return cost;
}


/// -----------------------------------------------------------
/// \brief Compute if an edge crosses another
/// -----------------------------------------------------------
inline int edgeCrossing(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4)
{
    if (x2 == x1)
    {
        if (x3 == x4)
        {
            if (x1 != x3) return 0;
            else if (std::max(y3, y4) < std::min(y1, y2) || std::max(y1, y2) < std::min(y3, y4)) return 0;
            else return 1;
        }
        else
        {
            double a2 = (y4 - y3) / (x4 - x3);
            double b2 = y3 - (a2 * x3);
            if (a2 == 0)
            {
                if (std::min(y1, y2) > b2 || std::max(y1, y2) < b2) return 0;
                else if (x2 <= std::max(x3, x4) && x2 >= std::min(x3, x4)) return 1;
                else return 0;
            }
            else if (a2*x1 + b2 <= std::min(std::max(y3, y4), std::max(y1, y2)) && a2*x1 + b2 >= std::max(std::min(y3, y4), std::min(y1, y2)))
                return 1;
            else return 0;
        }
    }
    else if (x3 == x4)
    {
        if( x1 == x2)
        {
            if (x1 != x3) return 0;
            else if (std::max(y3, y4) < std::min(y1, y2) || std::max(y1, y2) < std::min(y3, y4)) return 0;
            else return 1;
        }
        else
        {
            double a1 = (y2 - y1) / (x2 - x1);
            double b1 = y1 - (a1 * x1);
            if (a1 == 0)
            {
                if (std::min(y3, y4) > b1 || std::max(y3, y4) < b1) return 0;
                else if (x3 <= std::max(x1, x2) && x3 >= std::min(x1, x2)) return 1;
                else return 0;
            }
            else if (((a1*x3 + b1) <= std::min(std::max(y1, y2), std::max(y3, y4))) && ((a1*x3 + b1) >= std::max(std::min(y1, y2), std::min(y3, y4))))
                return 1;
            else return 0;
        }
    }
    else
    {
        double a1 = (y2 - y1) / (x2 - x1);
        double a2 = (y4 - y3) / (x4 - x3);
        if (a1 == a2) return 0;
        else
        {
            double b2 = y3 - (a2 * x3);
            double b1 = y1 - (a1 * x1);
            double xcommun = (b2 - b1) / (a1 - a2);
            if (xcommun >= std::max(std::min(x1, x2), std::min(x3, x4)) && xcommun <= std::min(std::max(x1, x2), std::max(x3, x4))) return 1;
            else return 0;
        }
    }
}


/// -----------------------------------------------------------
/// \brief Line intersection with the option of saving the
/// intersection point if it exists
/// -----------------------------------------------------------
inline int get_line_intersection(double p0_x, double p0_y, double p1_x, double p1_y,
    double p2_x, double p2_y, double p3_x, double p3_y, double *i_x, double *i_y)
{
    double s02_x, s02_y, s10_x, s10_y, s32_x, s32_y, s_numer, t_numer, denom, t;
    s10_x = p1_x - p0_x;
    s10_y = p1_y - p0_y;
    s32_x = p3_x - p2_x;
    s32_y = p3_y - p2_y;

    denom = s10_x * s32_y - s32_x * s10_y;
    if (denom == 0)
        return 0; // Collinear
    bool denomPositive = denom > 0;

    s02_x = p0_x - p2_x;
    s02_y = p0_y - p2_y;
    s_numer = s10_x * s02_y - s10_y * s02_x;
    if ((s_numer < 0) == denomPositive)
        return 0; // No collision

    t_numer = s32_x * s02_y - s32_y * s02_x;
    if ((t_numer < 0) == denomPositive)
        return 0; // No collision

    if (((s_numer > denom) == denomPositive) || ((t_numer > denom) == denomPositive))
        return 0; // No collision
    // Collision detected
    t = t_numer / denom;
    if (i_x != NULL)
        *i_x = p0_x + (t * s10_x);
    if (i_y != NULL)
        *i_y = p0_y + (t * s10_y);

    return 1;
}


/// -----------------------------------------------------------
/// \brief Distance between a point and a line
/// -----------------------------------------------------------
inline double point_dist(double x1, double y1, double x2, double y2, double x3, double y3)
{
    // x3, y3 is the point
    double px = x2-x1;
    double py = y2-y1;

    double something = px*px + py*py;

    double u = ((x3 - x1) * px + (y3 - y1) * py) / float(something);

    if (u > 1)
        u = 1;
    else if (u < 0)
        u = 0;

    double x = x1 + u * px;
    double y = y1 + u * py;

    double dx = x - x3;
    double dy = y - y3;

    double dist = sqrt(dx*dx + dy*dy);

    return dist;
}


inline std::pair<double, bool> distance2Segment(Array4D x, Array4D xs, Array4D xe)
{
    double xa = xs[0]; double ya = xs[1];
    double xb = xe[0]; double yb = xe[1];
    double xp = x[0];  double yp = x[1];

    // % x-coordinates
    double A = xb-xa;
    double B = yb-ya;
    double C = yp*B+xp*A;
    double a =  2*((B*B)+(A*A));
    double b = -4*A*C+(2*yp+ya+yb)*A*B-(2*xp+xa+xb)*(B*B);
    double c =  2*(C*C)-(2*yp+ya+yb)*C*B+(yp*(ya+yb)+xp*(xa+xb))*(B*B);
    double x1 = (-b + sqrt((b*b)-4*a*c))/(2*a);
    double x2 = (-b - sqrt((b*b)-4*a*c))/(2*a);

    // % y-coordinates
    A = yb-ya;
    B = xb-xa;
    C = xp*B+yp*A;
    a =  2*((B*B)+(A*A));
    b = -4*A*C+(2*xp+xa+xb)*A*B-(2*yp+ya+yb)*(B*B);
    c =  2*(C*C)-(2*xp+xa+xb)*C*B+(xp*(xa+xb)+yp*(ya+yb))*(B*B);
    double y1 = (-b + sqrt((b*b)-4*a*c))/(2*a);
    double y2 = (-b - sqrt((b*b)-4*a*c))/(2*a);

    // % Put point candidates together
    std::vector<double> dvec; dvec.reserve(4);
    Array4D xfm1 = {{x1,y1,0,0}};
    Array4D xfm2 = {{x2,y2,0,0}};
    Array4D xfm3 = {{x1,y2,0,0}};
    Array4D xfm4 = {{x2,y1,0,0}};

    dvec.push_back(edist(xfm1, x));
    dvec.push_back(edist(xfm2, x));
    dvec.push_back(edist(xfm3, x));
    dvec.push_back(edist(xfm4, x));

    // double dmax = *std::max_element(dvec.begin(), dvec.end());
    double dmax = -1;
    double imax = -1;
    for (int i = 0; i < 4; i++)
    {
        if (dvec[i] > dmax)
        {
            dmax = dvec[i];
            imax = i;
        }
    }

    Array4D xf;
    if (imax == 0)
        xf = xfm1;
    else if (imax == 1)
        xf = xfm2;
    else if (imax == 2)
        xf = xfm3;
    else if (imax == 3)
        xf = xfm4;

    //inside = (sign((xs-xf)'*(xe-xf))<=0);
    Array4D xs_xf = {{xs[0]-xf[0], xs[1]-xf[1], 0, 0}};
    Array4D xe_xf = {{xe[0]-xf[0], xe[1]-xf[1], 0, 0}};
    double dotp = (xs_xf[0] * xe_xf[0]) + (xs_xf[1] * xe_xf[1]);

    bool inside = false;
    if (dotp <= 0.0)
        inside = true;

    return std::make_pair(dmax, inside);
}



/// -----------------------------------------------------------
/// \brief Compute if an edge crosses another
/// -----------------------------------------------------------
inline double pathLength(std::vector<Array4D> traj)
{
    double plen = 0.0;
    for (size_t i = 0; i < traj.size()-1; i++)
    {
        plen += edist(traj[i], traj[i+1]);
    }
    return plen;
}


inline Array2D normalizeVector(const Array4D p1, const Array4D p2)
{
    Array2D v = {{p1[0]-p2[0], p1[1]-p2[1]}};

    double vlen = sqrt((v[0]*v[0] + v[1]*v[1]));
    if (vlen < 1e-12)
        return v;
    else
    {
        Array2D vv = {{v[0]/vlen, v[1]/vlen}};
        return vv;
    }
}


inline Array2D normalizeVector(const Array2D v)
{
    double vlen = sqrt((v[0]*v[0] + v[1]*v[1]));
    if (vlen < 1e-12)
        return v;
    else
    {
        Array2D vv = {{v[0]/vlen, v[1]/vlen}};
        return vv;
    }
}

/// -----------------------------------------------------------
/// \brief Get the angle between vectors
/// -----------------------------------------------------------
inline double vecAngle(const Array4D p1, const Array4D p2, const Array4D p3, const Array4D p4)
{
    Array2D nv1 = normalizeVector(p1, p2);
    Array2D nv2 = normalizeVector(p3, p4);

    double vdot = (nv1[0] * nv2[0]) + (nv1[1] * nv2[1]);
    double angle = acos(vdot);
    return angle;
}


/// -----------------------------------------------------------
/// \brief Rotate a 2D point about another point
/// -----------------------------------------------------------
inline Array4D rotate2d(const double theta,
                        const double px, const double py,
                        const double ox, const double oy,
                        bool clockwise=false)
{
    if (!clockwise)
    {
        double ppx = (cos(theta) * (px-ox) - sin(theta) * (py-oy)) + ox;
        double ppy = (sin(theta) * (px-ox) + cos(theta) * (py-oy)) + oy;
        Array4D rp = {{ppx, ppy, 0, 0}};
        return rp;
    }
    else
    {
        double ppx = (cos(theta) * (px-ox) + sin(theta) * (py-oy)) + ox;
        double ppy = (-sin(theta) * (px-ox) + cos(theta) * (py-oy)) + oy;
        Array4D rp = {{ppx, ppy, 0, 0}};
        return rp;
    }
}


/// -----------------------------------------------------------
/// \brief Get angle between a vector and horizontal axis
/// -----------------------------------------------------------
inline double angle_horizontal(const Array4D p1, const Array4D p2)
{
    // double dx = p2[0]-p1[0];

    // if (dx > 0.0)
    // {
    //     Array4D p3 = {{p1[0], p1[1], 0, 0}};
    //     Array4D p4 = {{p1[0]+1, p1[1], 0, 0}};

    //     double rel_theta = vecAngle(p1, p2, p3, p4);
    //     return rel_theta;
    // }
    // else
    // {
    //     Array4D p3 = {{p1[0], p1[1], 0, 0}};
    //     Array4D p4 = {{p1[0]-1, p1[1], 0, 0}};

    //     double rel_theta = vecAngle(p1, p2, p3, p4);
    //     return rel_theta;
    // }

    // TODO - add normalize
    // if (rel_theta > M_PI/2.0)
    //     rel_theta = M_PI - rel_theta;

    return atan2(p2[1]-p1[1], p2[0]-p1[0]);

}


/// -----------------------------------------------------------
/// \brief Evaluate a gaussian at a point
/// -----------------------------------------------------------
inline double eval_gaussian(double x, double mu, double sigma=0.2)
{
    double fg = (1.0 / (sigma * sqrt(2*M_PI))) * exp(-(x - mu)*(x - mu) / (2.0 * sigma * sigma));
    return fg / 1.0;
}


/// -----------------------------------------------------------
/// \brief Social relation feature (Gaussian around links)
/// -----------------------------------------------------------
inline double social_relation_feature(std::vector<Array4D> action, const Array4D r1, const Array4D r2,
                                      const double sigma=0.2, const double discount=0.99)
{
    double feature = 0.0;
    for (int i = 0; i < action.size()-1; i++)
    {
        Array4D x = {{action[i][0], action[i][1], 0, 0}};

        std::pair<double, bool> result = distance2Segment(x, r1, r2);

        if (result.second == 1)
        {
            feature += eval_gaussian(result.first, 0.0, sigma) * std::pow(discount, float(i));
        }
        else
        {
            double ed = std::min(edist(x, r1), edist(x, r2));
            feature += eval_gaussian(ed, 0.0, sigma) * std::pow(discount, float(i));
        }
    }

    return feature;
}



/// -----------------------------------------------------------
/// \brief Compute the social influece of an action point on
/// the pedestrians using the social force model
/// -----------------------------------------------------------
// inline double anisotropicDistance(const Array4D robot, const Array4D agent, const double dij=0.03,
//                                   const double ak=0.08267, const double bk=1.0, const double lambda=0.4,
//                                   const double rij=0.0133)
inline double anisotropicDistance(const Array4D robot, const Array4D agent, const double dij=1.0,
                                  const double ak=2.48, const double bk=1.0, const double lambda=0.4,
                                  const double rij=0.45)
{
    Array2D ei = {{-robot[2], -robot[3]}};
    ei = normalizeVector(ei);

    double phi = atan2(agent[1] - robot[1], agent[0] - robot[0]);
    Array2D nij = {{cos(phi), sin(phi)}};

    Array2D alpha = {{ak * exp((rij - dij) / bk) * nij[0], ak * exp((rij - dij) / bk) * nij[1]}};
    double beta = (lambda + (1-lambda)) * ((1 + cos(phi)) / 2.0);

    Array2D ap = {{alpha[0] * beta, alpha[1] * beta}};
    double dc = sqrt(ap[0]*ap[0] + ap[1]*ap[1]);
    return dc;
}


/// -----------------------------------------------------------
/// \brief Compute the feature of a trajectory and return a
/// 3D double array with edges_crossed, concentration and
/// goal deviation
/// -----------------------------------------------------------
// inline std::vector<double> computeTrajectoryFeatures(std::vector<Array4D> traj, std::vector<Array4D> people,
//                                                      std::vector<Array2I> relations, Array4D goal,
//                                                      double discount, double human_radius)


inline std::vector<double> computeTrajectoryFeatures(std::vector<Array4D> traj, std::map<int,Array4D> map_people,
                                                     std::vector<Array2I> relations, Array4D goal,
                                                     double discount, double human_radius)
{
    // --------------------------------------
    double srb = 0.0;
    for (int k = 0; k < relations.size(); k++)
    {
        Array2I rel = relations[k];
        size_t i = rel[0];
        size_t j = rel[1];
        
        // Array4D r1 = {{people[i-1][0], people[i-1][1], 0, 0}};
        // Array4D r2 = {{people[j-1][0], people[j-1][1], 0, 0}};

        Array4D r1 = {{map_people[i][0], map_people[i][1], 0, 0}};
        Array4D r2 = {{map_people[j][0], map_people[j][1], 0, 0}};
        srb += social_relation_feature(traj, r1, r2, 0.4, discount);

        // ROS_ERROR("%d, %d, (%f, %f), (%f, %f)", i, j, r1[0], r1[1], r2[0], r2[1]);
    }


    // --------------------------------------
    double concentration = 0.0;
    for (unsigned int i = 0; i < traj.size(); i++)
    {
        Array4D p = traj[i];

        // for (int j = 0; j < people.size(); j++)
        // {
        //     Array4D hp = people[j];

        //     double ed = edist(hp, p);
        //     double ad = anisotropicDistance(hp, p, ed);
        //     if (ed < ad)
        //         concentration += ad * std::pow(discount, float(i));
        // }


        std::map<int, Array4D>::iterator iter = map_people.begin();


        for (; iter != map_people.end(); iter++)
        {
            
            Array4D hp = iter->second;

            double ed = edist(hp, p);
            double ad = anisotropicDistance(hp, p, ed);
            if (ed < ad)
                concentration += ad * std::pow(discount, float(i));
        }



    }


    // ------------------------------------------
    double goal_deviation = 0.0;
    for (size_t i = 0; i < traj.size() - 1; i++)
    {
        goal_deviation += vecAngle(traj[i], traj[i + 1], traj[i], goal) * std::pow(discount, float(i));
    }
    // goal_deviation /= 50.0;  // scaling to be in the same range as the other features

    std::vector<double> feature = {{srb, concentration, goal_deviation}};

    return feature;
}



/// Used for handling relations to ensure only unique elements
typedef  std::pair<int, int> PairInt;



#endif
