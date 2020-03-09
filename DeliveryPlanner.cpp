#include "provided.h"
#include <vector>
using namespace std;

class DeliveryPlannerImpl
{
public:
    DeliveryPlannerImpl(const StreetMap* sm);
    ~DeliveryPlannerImpl();
    DeliveryResult generateDeliveryPlan(
        const GeoCoord& depot,
        const vector<DeliveryRequest>& deliveries,
        vector<DeliveryCommand>& commands,
        double& totalDistanceTravelled) const;
private:
    DeliveryOptimizer* dop;
    PointToPointRouter* ptp;
};

DeliveryPlannerImpl::DeliveryPlannerImpl(const StreetMap* sm)
{
    dop = new DeliveryOptimizer(sm);
    ptp = new PointToPointRouter(sm);
}

DeliveryPlannerImpl::~DeliveryPlannerImpl()
{
    delete dop;
    delete ptp;
}

DeliveryResult DeliveryPlannerImpl::generateDeliveryPlan(
    const GeoCoord& depot,
    const vector<DeliveryRequest>& deliveries,
    vector<DeliveryCommand>& commands,
    double& totalDistanceTravelled) const
{
    totalDistanceTravelled = 0;
    double tempDist;
    double oldCrowDistance, newCrowDistance;
    vector<DeliveryRequest> reorderedDel = deliveries;
    list<StreetSegment> route;
    dop->optimizeDeliveryOrder(depot, reorderedDel, oldCrowDistance, newCrowDistance);
    
    // DON'T FORGET TO DEAL WITH IF PTPROUTE RETURNS BAD_COORD
    // DEAL WITH THIS IN DELIVERY OPTIMIZER?
    
    if (ptp->generatePointToPointRoute(depot, reorderedDel[0].location, route, tempDist) != DELIVERY_SUCCESS)
    {
        // end here
    }
    totalDistanceTravelled += tempDist;
    GeoCoord prev = reorderedDel[0].location;
    for (int i=1; i<reorderedDel.size(); i++)
    {
        if (ptp->generatePointToPointRoute(prev, reorderedDel[i].location, route, tempDist) == DELIVERY_SUCCESS)
        {
            prev = reorderedDel[i].location;
            totalDistanceTravelled += tempDist;
            for (auto itr = route.begin(); itr != route.end(); itr++)
            {
                
            }
        }
    }
}

//******************** DeliveryPlanner functions ******************************

// These functions simply delegate to DeliveryPlannerImpl's functions.
// You probably don't want to change any of this code.

DeliveryPlanner::DeliveryPlanner(const StreetMap* sm)
{
    m_impl = new DeliveryPlannerImpl(sm);
}

DeliveryPlanner::~DeliveryPlanner()
{
    delete m_impl;
}

DeliveryResult DeliveryPlanner::generateDeliveryPlan(
    const GeoCoord& depot,
    const vector<DeliveryRequest>& deliveries,
    vector<DeliveryCommand>& commands,
    double& totalDistanceTravelled) const
{
    return m_impl->generateDeliveryPlan(depot, deliveries, commands, totalDistanceTravelled);
}

