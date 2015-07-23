////
// VelmaQ5Q6CollisionChecker.h
//
//  Created on: Jul 23, 2015
//      Author: dseredyn
////

#ifndef VELMAQ5Q6COLLISIONCHECKER_H_
#define VELMAQ5Q6COLLISIONCHECKER_H_

#include <octomap/OcTree.h>
#include <string>
#include <openrave/openrave.h>
#include <openrave/collisionchecker.h>
#include <ros/time.h>


namespace or_velma
{
    class VelmaQ5Q6CollisionChecker : public OpenRAVE::CollisionCheckerBase
    {
        public:
            VelmaQ5Q6CollisionChecker(OpenRAVE::EnvironmentBasePtr env);
            VelmaQ5Q6CollisionChecker(OpenRAVE::EnvironmentBasePtr env, OpenRAVE::CollisionCheckerBasePtr wrappedChecker);
            virtual ~VelmaQ5Q6CollisionChecker();
            virtual void  Clone (OpenRAVE::InterfaceBaseConstPtr preference, int cloningoptions);
            virtual bool SetCollisionOptions(int collisionoptions);
            virtual int GetCollisionOptions() const;
            virtual void SetTolerance(OpenRAVE::dReal tolerance);
            virtual bool InitKinBody(OpenRAVE::KinBodyPtr pbody);
            virtual bool CheckCollision(OpenRAVE::KinBodyConstPtr pbody1, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());
            virtual bool CheckCollision(OpenRAVE::KinBodyConstPtr pbody1, OpenRAVE::KinBodyConstPtr pbody2, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());
            virtual bool CheckCollision(OpenRAVE::KinBody::LinkConstPtr plink, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());
            virtual bool CheckCollision(OpenRAVE::KinBody::LinkConstPtr plink1, OpenRAVE::KinBody::LinkConstPtr plink2, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());
            virtual bool CheckCollision(OpenRAVE::KinBody::LinkConstPtr plink, OpenRAVE::KinBodyConstPtr pbody, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());
            virtual bool CheckCollision(OpenRAVE::KinBody::LinkConstPtr plink, const std::vector< OpenRAVE::KinBodyConstPtr > &vbodyexcluded, const std::vector< OpenRAVE::KinBody::LinkConstPtr > &vlinkexcluded, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());
            virtual bool CheckCollision(OpenRAVE::KinBodyConstPtr pbody, const std::vector< OpenRAVE::KinBodyConstPtr > &vbodyexcluded, const std::vector< OpenRAVE::KinBody::LinkConstPtr > &vlinkexcluded, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());
            virtual bool CheckCollision(const OpenRAVE::RAY &ray, OpenRAVE::KinBody::LinkConstPtr plink, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());
            virtual bool CheckCollision(const OpenRAVE::RAY &ray, OpenRAVE::KinBodyConstPtr pbody, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());
            virtual bool CheckCollision(const OpenRAVE::RAY &ray, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());

            /// \brief Checks self collision only with the links of the passed in body.
            ///
            /// Only checks KinBody::GetNonAdjacentLinks(), Links that are joined together are ignored.
            /// \param[in] pbody The body to check self-collision for
            /// \param[out] report [optional] collision report to be filled with data about the collision.
            virtual bool CheckStandaloneSelfCollision(OpenRAVE::KinBodyConstPtr body, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());

            /// \brief Checks self collision of the link with the rest of the links with its parent
            ///
            /// Only checks KinBody::GetNonAdjacentLinks(), Links that are joined together are ignored.
            /// \param[out] report [optional] collision report to be filled with data about the collision.
            virtual bool CheckStandaloneSelfCollision(OpenRAVE::KinBody::LinkConstPtr link, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());

            virtual bool InitEnvironment();
            virtual void DestroyEnvironment();
            virtual void RemoveKinBody(OpenRAVE::KinBodyPtr body);

            inline OpenRAVE::CollisionCheckerBasePtr GetWrappedChecker() { return m_wrappedChecker; }
            inline void SetWrappedChecker(OpenRAVE::CollisionCheckerBasePtr checker) { m_wrappedChecker = checker;}

        protected:
            bool CheckQ5Q6Collision(OpenRAVE::KinBodyConstPtr body, OpenRAVE::CollisionReportPtr report);
            bool PointInsidePolygon(double x, double y, const std::vector<std::pair<double, double> > &poly) const;

            OpenRAVE::CollisionCheckerBasePtr m_wrappedChecker;
            std::vector<std::pair<double, double> > m_q5q6polygon;
    };

} /* namespace or_velma */
#endif /* VELMAQ5Q6COLLISIONCHECKER_H_ */
