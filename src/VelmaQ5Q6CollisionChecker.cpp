////
// VelmaQ5Q6CollisionChecker.cpp
//
//  Created on: Jul 23, 2015
//      Author: dseredyn
////

#include "VelmaQ5Q6CollisionChecker.h"

namespace or_velma
{
    
    VelmaQ5Q6CollisionChecker::VelmaQ5Q6CollisionChecker(OpenRAVE::EnvironmentBasePtr env) :
            CollisionCheckerBase(env)
    {
            m_q5q6polygon.push_back(std::make_pair(-0.397855401039,-2.90307354927));
            m_q5q6polygon.push_back(std::make_pair(2.12894010544,-2.90307354927));
            m_q5q6polygon.push_back(std::make_pair(2.12043237686,-1.87363839149));
            m_q5q6polygon.push_back(std::make_pair(1.92475450039,-1.43123674393));
            m_q5q6polygon.push_back(std::make_pair(0.77621114254,-1.39720571041));
            m_q5q6polygon.push_back(std::make_pair(0.350824713707,-1.00585031509));
            m_q5q6polygon.push_back(std::make_pair(0.401871085167,-0.571956157684));
            m_q5q6polygon.push_back(std::make_pair(0.810242056847,0.414940297604));
            m_q5q6polygon.push_back(std::make_pair(1.34622907639,0.942419290543));
            m_q5q6polygon.push_back(std::make_pair(2.11192464828,1.01898884773));
            m_q5q6polygon.push_back(std::make_pair(2.12894010544,2.8906891346));
            m_q5q6polygon.push_back(std::make_pair(-0.814733862877,2.8906891346));
            m_q5q6polygon.push_back(std::make_pair(-1.22310483456,2.27813267708));
            m_q5q6polygon.push_back(std::make_pair(-2.21850919724,2.29514837265));
            m_q5q6polygon.push_back(std::make_pair(-2.22701668739,-1.32063627243));
            m_q5q6polygon.push_back(std::make_pair(-1.81013822556,-1.66945314407));
            m_q5q6polygon.push_back(std::make_pair(-0.814733862877,-1.73751521111));
            m_q5q6polygon.push_back(std::make_pair(-0.423378348351,-2.09483933449));
    }

    VelmaQ5Q6CollisionChecker::VelmaQ5Q6CollisionChecker(OpenRAVE::EnvironmentBasePtr env, OpenRAVE::CollisionCheckerBasePtr wrappedChecker) :
            CollisionCheckerBase(env),
            m_wrappedChecker(wrappedChecker)
    {
            m_q5q6polygon.push_back(std::make_pair(-0.397855401039,-2.90307354927));
            m_q5q6polygon.push_back(std::make_pair(2.12894010544,-2.90307354927));
            m_q5q6polygon.push_back(std::make_pair(2.12043237686,-1.87363839149));
            m_q5q6polygon.push_back(std::make_pair(1.92475450039,-1.43123674393));
            m_q5q6polygon.push_back(std::make_pair(0.77621114254,-1.39720571041));
            m_q5q6polygon.push_back(std::make_pair(0.350824713707,-1.00585031509));
            m_q5q6polygon.push_back(std::make_pair(0.401871085167,-0.571956157684));
            m_q5q6polygon.push_back(std::make_pair(0.810242056847,0.414940297604));
            m_q5q6polygon.push_back(std::make_pair(1.34622907639,0.942419290543));
            m_q5q6polygon.push_back(std::make_pair(2.11192464828,1.01898884773));
            m_q5q6polygon.push_back(std::make_pair(2.12894010544,2.8906891346));
            m_q5q6polygon.push_back(std::make_pair(-0.814733862877,2.8906891346));
            m_q5q6polygon.push_back(std::make_pair(-1.22310483456,2.27813267708));
            m_q5q6polygon.push_back(std::make_pair(-2.21850919724,2.29514837265));
            m_q5q6polygon.push_back(std::make_pair(-2.22701668739,-1.32063627243));
            m_q5q6polygon.push_back(std::make_pair(-1.81013822556,-1.66945314407));
            m_q5q6polygon.push_back(std::make_pair(-0.814733862877,-1.73751521111));
            m_q5q6polygon.push_back(std::make_pair(-0.423378348351,-2.09483933449));
    }
    
    VelmaQ5Q6CollisionChecker::~VelmaQ5Q6CollisionChecker()
    {
    }

    void VelmaQ5Q6CollisionChecker::Clone(OpenRAVE::InterfaceBaseConstPtr preference, int cloningoptions)
    {
        CollisionCheckerBase::Clone(preference, cloningoptions);

        const VelmaQ5Q6CollisionChecker* otherChecker = dynamic_cast<const VelmaQ5Q6CollisionChecker*>(preference.get());

        if(!otherChecker)
        {
            return;
        }

        if(m_wrappedChecker)
        {
            m_wrappedChecker->Clone(otherChecker->m_wrappedChecker, cloningoptions);
        }
        else
        {
            m_wrappedChecker = OpenRAVE::RaveCreateCollisionChecker(GetEnv(), otherChecker->m_wrappedChecker->GetXMLId());
            m_wrappedChecker->Clone(otherChecker->m_wrappedChecker, cloningoptions);
        }
    }

    bool VelmaQ5Q6CollisionChecker::SetCollisionOptions(int collisionoptions)
    {
        return m_wrappedChecker->SetCollisionOptions(collisionoptions);
    }

    int VelmaQ5Q6CollisionChecker::GetCollisionOptions() const
    {
        return m_wrappedChecker->GetCollisionOptions();
    }

    void VelmaQ5Q6CollisionChecker::SetTolerance(OpenRAVE::dReal tolerance)
    {
        return m_wrappedChecker->SetTolerance(tolerance);
    }

    bool VelmaQ5Q6CollisionChecker::InitKinBody(OpenRAVE::KinBodyPtr pbody)
    {
        return m_wrappedChecker->InitKinBody(pbody);
    }

    bool VelmaQ5Q6CollisionChecker::CheckCollision(OpenRAVE::KinBodyConstPtr pbody1, OpenRAVE::CollisionReportPtr report)
    {
        return m_wrappedChecker->CheckCollision(pbody1, report);
    }

    bool VelmaQ5Q6CollisionChecker::CheckCollision(OpenRAVE::KinBodyConstPtr pbody1, OpenRAVE::KinBodyConstPtr pbody2, OpenRAVE::CollisionReportPtr report)
    {
        return m_wrappedChecker->CheckCollision(pbody1, pbody2, report);
    }

    bool VelmaQ5Q6CollisionChecker::CheckCollision(OpenRAVE::KinBody::LinkConstPtr plink, OpenRAVE::CollisionReportPtr report)
    {
        return m_wrappedChecker->CheckCollision(plink, report);
    }

    bool VelmaQ5Q6CollisionChecker::CheckCollision(OpenRAVE::KinBody::LinkConstPtr plink1, OpenRAVE::KinBody::LinkConstPtr plink2, OpenRAVE::CollisionReportPtr report)
    {
        return m_wrappedChecker->CheckCollision(plink1, plink2, report);
    }

    bool VelmaQ5Q6CollisionChecker::CheckCollision(OpenRAVE::KinBody::LinkConstPtr plink, OpenRAVE::KinBodyConstPtr pbody, OpenRAVE::CollisionReportPtr report)
    {
        return m_wrappedChecker->CheckCollision(plink, pbody, report);
    }

    bool VelmaQ5Q6CollisionChecker::CheckCollision(OpenRAVE::KinBody::LinkConstPtr plink, const std::vector< OpenRAVE::KinBodyConstPtr > &vbodyexcluded, const std::vector< OpenRAVE::KinBody::LinkConstPtr > &vlinkexcluded, OpenRAVE::CollisionReportPtr report)
    {
        return m_wrappedChecker->CheckCollision(plink, vbodyexcluded, vlinkexcluded, report);
    }

    bool VelmaQ5Q6CollisionChecker::CheckCollision(OpenRAVE::KinBodyConstPtr pbody, const std::vector< OpenRAVE::KinBodyConstPtr > &vbodyexcluded, const std::vector< OpenRAVE::KinBody::LinkConstPtr > &vlinkexcluded, OpenRAVE::CollisionReportPtr report)
    {
        return m_wrappedChecker->CheckCollision(pbody, vbodyexcluded, vlinkexcluded, report);
    }

    bool VelmaQ5Q6CollisionChecker::CheckCollision(const OpenRAVE::RAY &ray, OpenRAVE::KinBody::LinkConstPtr plink, OpenRAVE::CollisionReportPtr report)
    {
        return m_wrappedChecker->CheckCollision(ray, plink, report);
    }

    bool VelmaQ5Q6CollisionChecker::CheckCollision(const OpenRAVE::RAY &ray, OpenRAVE::KinBodyConstPtr pbody, OpenRAVE::CollisionReportPtr report)
    {
        return m_wrappedChecker->CheckCollision(ray, pbody, report);
    }

    bool VelmaQ5Q6CollisionChecker::CheckCollision(const OpenRAVE::RAY &ray, OpenRAVE::CollisionReportPtr report)
    {
        return m_wrappedChecker->CheckCollision(ray, report);
    }

    bool VelmaQ5Q6CollisionChecker::CheckStandaloneSelfCollision(OpenRAVE::KinBodyConstPtr body, OpenRAVE::CollisionReportPtr report)
    {
        if (CheckQ5Q6Collision(body, report))
        {
            return true;
        }
        return m_wrappedChecker->CheckStandaloneSelfCollision(body, report);
    }

    bool VelmaQ5Q6CollisionChecker::CheckStandaloneSelfCollision(OpenRAVE::KinBody::LinkConstPtr link, OpenRAVE::CollisionReportPtr report)
    {
        OpenRAVE::KinBodyConstPtr body = link->GetParent();
        if (CheckQ5Q6Collision(body, report))
        {
            return true;
        }
        return m_wrappedChecker->CheckStandaloneSelfCollision(body, report);
    }

    bool VelmaQ5Q6CollisionChecker::InitEnvironment()
    {
        return m_wrappedChecker->InitEnvironment();
    }

    void VelmaQ5Q6CollisionChecker::DestroyEnvironment()
    {
        return m_wrappedChecker->DestroyEnvironment();
    }


    void VelmaQ5Q6CollisionChecker::RemoveKinBody(OpenRAVE::KinBodyPtr body)
    {
        m_wrappedChecker->RemoveKinBody(body);
    }

    bool VelmaQ5Q6CollisionChecker::CheckQ5Q6Collision(OpenRAVE::KinBodyConstPtr body, OpenRAVE::CollisionReportPtr report)
    {
        OpenRAVE::KinBody::JointPtr joint_l5 = body->GetJoint("left_arm_5_joint");
        OpenRAVE::KinBody::JointPtr joint_l6 = body->GetJoint("left_arm_6_joint");
        OpenRAVE::KinBody::JointPtr joint_r5 = body->GetJoint("right_arm_5_joint");
        OpenRAVE::KinBody::JointPtr joint_r6 = body->GetJoint("right_arm_6_joint");

        std::vector< OpenRAVE::dReal > dof;
        body->GetDOFValues(dof);

        if (joint_l5.get() != NULL && joint_l6.get() != NULL)
        {
            if (!PointInsidePolygon(-dof[joint_l5->GetDOFIndex()], -dof[joint_l6->GetDOFIndex()], m_q5q6polygon))
            {
                return true;
            }
        }

        if (joint_r5.get() != NULL && joint_r6.get() != NULL)
        {
            if (!PointInsidePolygon(dof[joint_r5->GetDOFIndex()], dof[joint_r6->GetDOFIndex()], m_q5q6polygon))
            {
                return true;
            }
        }
        return false;
    }

    // determine if a point is inside a given polygon or not
    // Polygon is a list of (x,y) pairs.
    bool VelmaQ5Q6CollisionChecker::PointInsidePolygon(double x, double y, const std::vector<std::pair<double, double> > &poly) const
    {
        int n = poly.size();
        bool inside = false;
        double p1x = poly[0].first, p1y = poly[0].second;
        for (int i=1; i < n+1; i++)
        {
            double p2x = poly[ i % n ].first, p2y = poly[ i % n ].second;
//            if (y > min(p1y,p2y))
            if (y > p1y || y > p2y)
            {
//                if (y <= max(p1y,p2y))
                if (y <= p1y || y <= p2y)
                {
//                    if (x <= max(p1x,p2x))
                    if (x <= p1x || x <= p2x)
                    {
                        double xinters = 0.0;
                        if (p1y != p2y)
                        {
                            xinters = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x;
                        }
                        if (p1x == p2x || x <= xinters)
                        {
                            inside = !inside;
                        }
                    }
                }
            }
            p1x = p2x;
            p1y = p2y;
        }

//        std::cout << "VelmaQ5Q6CollisionChecker::PointInsidePolygon " << x << " " << y << " " << (inside?"true":"false") << std::endl;

        return inside;
    }

} /* namespace or_velma */
