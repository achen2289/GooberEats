#include "provided.h"
#include "ExpandableHashMap.h"
#include <queue>
#include <list>
#include <cmath>
using namespace std;

class PointToPointRouterImpl
{
public:
    PointToPointRouterImpl(const StreetMap* sm);
    ~PointToPointRouterImpl();
    DeliveryResult generatePointToPointRoute(
        const GeoCoord& start,
        const GeoCoord& end,
        list<StreetSegment>& route,
        double& totalDistanceTravelled) const;
private:
    const StreetMap* m_sm;
    
    // gcPair used to place GeoCoords into priority queue based on an empirical and heuristic value
    struct gcPair
    {
        // gcPair takes in current GC, previous GC, and final GC
        gcPair(GeoCoord current, GeoCoord previous, GeoCoord finalDest)
        : curr(current), prev(previous), final(finalDest)
        {}
        GeoCoord curr;
        GeoCoord prev;
        GeoCoord final;
        
        // operator > overload to allow priority queue to sort in increasing order
        bool operator>(const gcPair& other) const
        {
            // g value is distance from previous gc to current gc
            // h value is distance from current gc to final destination
            double g_this = distanceEarthMiles(this->prev, this->curr);
            double h_this = distanceEarthMiles(this->curr, this->final);
            double g_other = distanceEarthMiles(other.prev, other.curr);
            double h_other = distanceEarthMiles(other.curr, other.final);
            return (g_this + h_this) > (g_other + h_other); // compare sum of g and h values
        }
    };
};

PointToPointRouterImpl::PointToPointRouterImpl(const StreetMap* sm)
{
    m_sm = sm;
}

PointToPointRouterImpl::~PointToPointRouterImpl()
{
}

DeliveryResult PointToPointRouterImpl::generatePointToPointRoute(
        const GeoCoord& start,
        const GeoCoord& end,
        list<StreetSegment>& route,
        double& totalDistanceTravelled) const
{
    totalDistanceTravelled = 0;
    for (auto itr = route.begin(); itr != route.end(); ) // clear route
        itr = route.erase(itr);
    if (start == end)
        return DELIVERY_SUCCESS;
    
    priority_queue<gcPair, vector<gcPair>, greater<gcPair>> pq; // priority queue to hold gcPairs
    ExpandableHashMap<GeoCoord, StreetSegment> previousSS; // maps ending GC to SS that led there
    vector<StreetSegment> segs; // holds street segments that begin with certain gc
    
    // end geocoord not found
    if (!m_sm->getSegmentsThatStartWith(end, segs))
        return BAD_COORD;
    
    // starting geocoord not found
    if (!m_sm->getSegmentsThatStartWith(start, segs))
        return BAD_COORD;
    
    pq.push(gcPair(start, start, end)); // push first GC
    while (!pq.empty())
    {
        gcPair gcP = pq.top();
        pq.pop();
        if (m_sm->getSegmentsThatStartWith(gcP.curr, segs))
        {
            // segs contains all street segments starting with gcP.curr
            for (auto itr = segs.begin(); itr != segs.end(); itr++)
            {
                if (previousSS.find((*itr).end) == nullptr) // if end GC has not been traveled to yet
                {
                    previousSS.associate((*itr).end, *itr); // associate ending GC with SS that led there
                    pq.push(gcPair((*itr).end, gcP.curr, end)); // push gcPair containing next GC, prev GC, and ending location
                    if ((*itr).end == end)
                    {
                        StreetSegment* prev = previousSS.find((*itr).end); // find SS leading to end GC
                        
                        // loop through street segments leading to final destination
                        while (prev->start != start)
                        {
                            route.push_front(*prev);
                            totalDistanceTravelled += distanceEarthMiles(prev->end, prev->start);
                            prev = previousSS.find(prev->start);
                        }
                        return DELIVERY_SUCCESS;
                    }
                }
            }
        }
    }
    return NO_ROUTE; // start geocoord does not lead to anywhere
}

//******************** PointToPointRouter functions ***************************

// These functions simply delegate to PointToPointRouterImpl's functions.
// You probably don't want to change any of this code.

PointToPointRouter::PointToPointRouter(const StreetMap* sm)
{
    m_impl = new PointToPointRouterImpl(sm);
}

PointToPointRouter::~PointToPointRouter()
{
    delete m_impl;
}

DeliveryResult PointToPointRouter::generatePointToPointRoute(
        const GeoCoord& start,
        const GeoCoord& end,
        list<StreetSegment>& route,
        double& totalDistanceTravelled) const
{
    return m_impl->generatePointToPointRoute(start, end, route, totalDistanceTravelled);
}
