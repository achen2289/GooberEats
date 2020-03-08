#include "provided.h"
#include "ExpandableHashMap.h"
#include <queue>
#include <list>
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
    StreetMap* sm;
};

PointToPointRouterImpl::PointToPointRouterImpl(const StreetMap* sm)
{
    sm = sm;
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
    if (start == end)
    {
        totalDistanceTravelled = 0;
        for (auto itr = route.begin(); itr != route.end(); )
            itr = route.erase(itr);
        return DELIVERY_SUCCESS;
    }
    
    GeoCoord curr = start;
    ExpandableHashMap<GeoCoord, StreetSegment> previousSS;
    queue<GeoCoord> points;
    vector<StreetSegment> segs;
    
    // get all street segments that start with starting GeoCoord
    
    // end coord not found
    if (!sm->getSegmentsThatStartWith(end, segs))
        return BAD_COORD;
    
    // searching and queueing up SS for first GC
    if (sm->getSegmentsThatStartWith(curr, segs))
    {
        // associate all end GeoCoords with the starting GeoCoord
        // queue up all end GeoCoords to be processed
        for (auto itr = segs.begin(); itr != segs.end(); itr++)
        {
            previousSS.associate((*itr).end, *itr); // key: ending GC. value: SS to get to ending GC
            points.push((*itr).end); // queue up end SS
            
            // if ending GC of a street segment is the end point
            if ((*itr).end == end)
            {
                route.push_front((*itr)); // push current SS
                StreetSegment* prev = previousSS.find((*itr).start); // find previous SS
                
                // repeat until starting GC of street segment is starting point
                while (prev != nullptr)
                {
                    route.push_front(*prev); // push prev SS to front
                    prev = previousSS.find((*prev).start); // look for starting GC of prev SS
                }
                return DELIVERY_SUCCESS;
            }
        }
        curr = points.front();
        points.pop();
    }
    else // if first GC not found
        return BAD_COORD;
    
    while (!points.empty())
    {
        // associate all end GeoCoords with the starting GeoCoord
        // queue up all end GeoCoords to be processed
        for (auto itr = segs.begin(); itr != segs.end(); itr++)
        {
            previousSS.associate((*itr).end, *itr); // key: ending GC. value: SS to get to ending GC
            points.push((*itr).end); // queue up end SS
            
            // if ending GC of a street segment is the end point
            if ((*itr).end == end)
            {
                route.push_front((*itr)); // push current SS
                
                // find previous SS
                StreetSegment* prev = previousSS.find((*itr).start);
                
                // repeat until starting GC of street segment is starting point
                while (prev != nullptr)
                {
                    route.push_front(*prev); // push prev SS to front
                    prev = previousSS.find((*prev).start); // look for starting GC of prev SS
                }
                return DELIVERY_SUCCESS;
            }
        }
        curr = points.front();
        points.pop();
    }
    return NO_ROUTE;
    
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
