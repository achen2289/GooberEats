#include "provided.h"
#include <vector>
#include <list>
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
    bool routeToCommand(list<StreetSegment>& route, vector<DeliveryCommand>& commands);
    string directionString(double angle);
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
    
    DeliveryResult dr = ptp->generatePointToPointRoute(depot, reorderedDel[0].location, route, tempDist);
    if (dr != DELIVERY_SUCCESS)
        return dr;
    
    totalDistanceTravelled += tempDist; // distance traveled to first delivery
    GeoCoord prev = reorderedDel[0].location; // store previous location GeoCoord
    
    // iterate through remaining deliveries
    for (int i=1; i<reorderedDel.size(); i++)
    {
        dr = ptp->generatePointToPointRoute(prev, reorderedDel[i].location, route, tempDist);
        if (dr == DELIVERY_SUCCESS)
        {
            prev = reorderedDel[i].location;
            totalDistanceTravelled += tempDist;
            for (auto itr = route.begin(); itr != route.end(); itr++)
            {
                
            }
        }
        else
            return dr;
    }
    return dr; // dr should contain DELIVERY_SUCESS
}

bool DeliveryPlannerImpl::routeToCommand(list<StreetSegment>& route, vector<DeliveryCommand>& commands)
{
    auto itr = route.begin(); // first street segment
    string currStr = (*itr).name; // current street name
    StreetSegment prevSS = *itr; // holds previous SS
    
    DeliveryCommand com1; // holds first delivery command
    com1.initAsProceedCommand(directionString(angleOfLine(*itr)), currStr, distanceEarthMiles((*itr).start, (*itr).end)); // first command put in
    
    itr++;
    
    // runs and increments distance to travel until different street reached
    while (itr != route.end() && (*itr).name == currStr)
    {
        com1.increaseDistance(distanceEarthMiles((*itr).start, (*itr).end));
        prevSS = *itr;
        itr++;
    }
    currStr = (*itr).name; // maintains street name
    commands.push_back(com1);
     
    // iterates through SS with 2nd street name until last SS
    while (itr != route.end())
    {
        DeliveryCommand com; // holds any delivery command
        double angleBet = angleBetween2Lines(prevSS, *itr);
        if (angleBet < 1 || angleBet > 359) // prev always first argument
        {
            com.initAsProceedCommand(directionString(angleOfLine(*itr)), currStr, distanceEarthMiles((*itr).start, (*itr).end));
            prevSS = *itr;
            itr++;
            while (itr != route.end() && (*itr).name == currStr)
            {
                com.increaseDistance(distanceEarthMiles((*itr).start, (*itr).end));
                prevSS = *itr;
                itr++;
            }
            currStr = (*itr).name;
            commands.push_back(com);
        }
        else if (angleBet >= 1 && angleBet < 180)
        {
            com.initAsTurnCommand("left", currStr);
            commands.push_back(com);
            DeliveryCommand com2;
            com2.initAsProceedCommand(directionString(angleOfLine(*itr)), currStr, distanceEarthMiles((*itr).start, (*itr).end));
        }
        else // if (angleBet >= 180 && angleBet <= 359)
        {
            com.initAsTurnCommand("right", currStr);
            commands.push_back(com);
        }
        
        if ((*itr).name == currStr)
        {
            while (itr != route.end() && (*itr).name == currStr)
            {
                com.increaseDistance(distanceEarthMiles((*itr).start, (*itr).end));
                itr++;
            }
        }
        else
            itr++;
        commands.push_back(com);
        currStr = (*itr).name; // new street name
//        itr++;
    }
    return true;
}

string DeliveryPlannerImpl::directionString(double angle)
{
    if (angle >= 0 && angle < 22.5)
        return "east";
    if (angle >= 22.5 && angle < 67.5)
        return "northeast";
    if (angle >= 67.5 && angle < 112.5)
        return "north";
    if (angle >= 112.5 && angle < 157.5)
        return "northwest";
    if (angle >= 157.5 && angle < 202.5)
        return "west";
    if (angle >= 202.5 && angle < 247.5)
        return "southwest";
    if (angle >= 247.5 && angle < 292.5)
        return "south";
    if (angle >= 292.5 && angle < 337.5)
        return "southeast";
    if (angle >= 337.5)
        return "east";
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

