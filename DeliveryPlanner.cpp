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
    bool routeToCommand(list<StreetSegment>& route, vector<DeliveryCommand>& commands) const;
    string directionString(double angle) const;
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
    for (auto itr = commands.begin(); itr != commands.end(); itr++) // clear commands
        itr = commands.erase(itr);
    
    double tempDist;
    double oldCrowDistance, newCrowDistance;
    vector<DeliveryRequest> reorderedDel = deliveries; // modifiable delivery request vector
    dop->optimizeDeliveryOrder(depot, reorderedDel, oldCrowDistance, newCrowDistance);
    
    // DON'T FORGET TO DEAL WITH IF PTPROUTE RETURNS BAD_COORD
    // DEAL WITH THIS IN DELIVERY OPTIMIZER?
    
    list<StreetSegment> route;
    DeliveryResult dr = ptp->generatePointToPointRoute(depot, reorderedDel[0].location, route, tempDist); // clears route before pushing back
    if (dr != DELIVERY_SUCCESS)
        return dr;
    routeToCommand(route, commands); // commands to get to first location
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
            routeToCommand(route, commands);
        }
        else
            return dr;
        
        // push back delivery command once destination has been reached
        DeliveryCommand dCom;
        dCom.initAsDeliverCommand(reorderedDel[i].item);
        commands.push_back(dCom);
    }
    
    // route from last location back to depot
    dr = ptp->generatePointToPointRoute(reorderedDel[reorderedDel.size()-1].location, depot, route, tempDist);
    if (dr != DELIVERY_SUCCESS)
        return dr;
    
    totalDistanceTravelled += tempDist;
    routeToCommand(route, commands); // route & commands back to depot
    return dr; // dr should contain DELIVERY_SUCESS
}

bool DeliveryPlannerImpl::routeToCommand(list<StreetSegment>& route, vector<DeliveryCommand>& commands) const
{
    string previous = "";
    auto itr = route.begin();
    while (itr != route.end())
    {
        // first street segment must create new proceed command
        if (itr == route.begin())
        {
            DeliveryCommand com;
            double angle = angleOfLine(*itr);
            double dist = distanceEarthMiles((*itr).start, (*itr).end);
            com.initAsProceedCommand(directionString(angle), (*itr).name, dist);
            previous = (*itr).name; // name of first SS
            itr++;
            
            while (itr != route.end() && (*itr).name == previous)
            {
                double dist = distanceEarthMiles((*itr).start, (*itr).end);
                com.increaseDistance(dist);
                previous = (*itr).name; // name of SS
                itr++;
            }
            commands.push_back(com);
        }
        
        // commands following first SS command
        if (itr != route.end() && (*itr).name != previous)
        {
            DeliveryCommand comm;
            auto itr2 = itr;
            itr2--; // iterator to previous SS
            double angleBet = angleBetween2Lines(*itr2, *itr);
            if (angleBet < 1 || angleBet > 359)
            {
                double angle = angleOfLine(*itr);
                double dist = distanceEarthMiles((*itr).start, (*itr).end);
                comm.initAsProceedCommand(directionString(angle), (*itr).name, dist);
                previous = (*itr).name;
                itr++;
            }
            else if (angleBet >= 1 && angleBet < 180)
            {
                DeliveryCommand commT;
                commT.initAsTurnCommand("left", (*itr).name);
                commands.push_back(commT);
                
                double angle = angleOfLine(*itr);
                double dist = distanceEarthMiles((*itr).start, (*itr).end);
                comm.initAsProceedCommand(directionString(angle), (*itr).name, dist);
                previous = (*itr).name;
                itr++;
            }
            else // if (angleBet >= 180 && angleBet <= 359)
            {
                DeliveryCommand commT;
                commT.initAsTurnCommand("right", (*itr).name);
                commands.push_back(commT);
                
                double angle = angleOfLine(*itr);
                double dist = distanceEarthMiles((*itr).start, (*itr).end);
                comm.initAsProceedCommand(directionString(angle), ((*itr).name), dist);
                previous = (*itr).name;
                itr++;
            }
            while (itr != route.end() && (*itr).name == previous)
            {
                double dist = distanceEarthMiles((*itr).start, (*itr).end);
                comm.increaseDistance(dist);
                previous = (*itr).name; // name of SS
                itr++;
            }
            commands.push_back(comm);
        }
    }
    return true;
}

string DeliveryPlannerImpl::directionString(double angle) const
{
    if (angle >= 0 && angle < 22.5)
        return "east";
    else if (angle >= 22.5 && angle < 67.5)
        return "northeast";
    else if (angle >= 67.5 && angle < 112.5)
        return "north";
    else if (angle >= 112.5 && angle < 157.5)
        return "northwest";
    else if (angle >= 157.5 && angle < 202.5)
        return "west";
    else if (angle >= 202.5 && angle < 247.5)
        return "southwest";
    else if (angle >= 247.5 && angle < 292.5)
        return "south";
    else if (angle >= 292.5 && angle < 337.5)
        return "southeast";
    else // (angle >= 337.5)
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

