#include "provided.h"
#include <vector>
using namespace std;

class DeliveryOptimizerImpl
{
public:
    DeliveryOptimizerImpl(const StreetMap* sm);
    ~DeliveryOptimizerImpl();
    void optimizeDeliveryOrder(
        const GeoCoord& depot,
        vector<DeliveryRequest>& deliveries,
        double& oldCrowDistance,
        double& newCrowDistance) const;
private:
    const StreetMap* m_sm;
};

DeliveryOptimizerImpl::DeliveryOptimizerImpl(const StreetMap* sm)
{
    m_sm = sm;
}

DeliveryOptimizerImpl::~DeliveryOptimizerImpl()
{
}

void DeliveryOptimizerImpl::optimizeDeliveryOrder(
    const GeoCoord& depot,
    vector<DeliveryRequest>& deliveries,
    double& oldCrowDistance,
    double& newCrowDistance) const
{
    oldCrowDistance = 0;
    newCrowDistance = 0;
    
    // compute oldCrowDistance
    oldCrowDistance += distanceEarthMiles(depot, deliveries[0].location);
    for (int i=1; i<deliveries.size(); i++)
    {
        oldCrowDistance += distanceEarthMiles(deliveries[i-1].location, deliveries[i].location);
    }
    
    vector<DeliveryRequest> tempDeliveries; // empty
    swap(tempDeliveries, deliveries); // deliveries now empty, temp contains deliveries elements
    
    GeoCoord starting = depot;
    while (!tempDeliveries.empty())
    {
        auto itrTemp = tempDeliveries.begin(); // current delivery request iterator
        double distBetw = distanceEarthMiles(starting, (*itrTemp).location); // dist from starting location to first location
        for (auto itr = tempDeliveries.begin(); itr != tempDeliveries.end(); itr++)
        {
            double dist = distanceEarthMiles(starting, (*itr).location);
            if (dist < distBetw) // if distance from starting location to any location is less, then set itrTemp to that location
                itrTemp = itr;
        }
        deliveries.push_back(*itrTemp); // add closest delivery request to deliveries
        starting = (*itrTemp).location; // set starting to GeoCoord of most recently added location
        tempDeliveries.erase(itrTemp); // delete that request from tempDeliveries
    }
    
    // compute newCrowDistance
    newCrowDistance += distanceEarthMiles(depot, deliveries[0].location);
    for (int i=1; i<deliveries.size(); i++)
    {
        newCrowDistance += distanceEarthMiles(deliveries[i-1].location, deliveries[i].location);
    }
}

//******************** DeliveryOptimizer functions ****************************

// These functions simply delegate to DeliveryOptimizerImpl's functions.
// You probably don't want to change any of this code.

DeliveryOptimizer::DeliveryOptimizer(const StreetMap* sm)
{
    m_impl = new DeliveryOptimizerImpl(sm);
}

DeliveryOptimizer::~DeliveryOptimizer()
{
    delete m_impl;
}

void DeliveryOptimizer::optimizeDeliveryOrder(
        const GeoCoord& depot,
        vector<DeliveryRequest>& deliveries,
        double& oldCrowDistance,
        double& newCrowDistance) const
{
    return m_impl->optimizeDeliveryOrder(depot, deliveries, oldCrowDistance, newCrowDistance);
}

