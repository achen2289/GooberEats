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
    const StreetMap* m_sm;
};

PointToPointRouterImpl::PointToPointRouterImpl(const StreetMap* sm)
{
    m_sm = sm;
}

PointToPointRouterImpl::~PointToPointRouterImpl()
{
    delete m_sm;
}

DeliveryResult PointToPointRouterImpl::generatePointToPointRoute(
        const GeoCoord& start,
        const GeoCoord& end,
        list<StreetSegment>& route,
        double& totalDistanceTravelled) const
{
    totalDistanceTravelled = 0;
    for (auto itr = route.begin(); itr != route.end(); )
        itr = route.erase(itr);
    if (start == end)
        return DELIVERY_SUCCESS;
    
    GeoCoord curr = start;
    ExpandableHashMap<GeoCoord, StreetSegment> previousSS;
    queue<GeoCoord> points;
    vector<StreetSegment> segs;
    
    // get all street segments that start with starting GeoCoord
    
    // end coord not found
    GeoCoord end2 = end;
    if (!m_sm->getSegmentsThatStartWith(end2, segs))
        return BAD_COORD;
    
    totalDistanceTravelled = 0;
    
    // searching and queueing up SS for first GC
    if (m_sm->getSegmentsThatStartWith(curr, segs))
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
                totalDistanceTravelled += distanceEarthMiles((*itr).end, (*itr).start); // distance of last SS
                StreetSegment* prev = previousSS.find((*itr).start); // find previous SS
                
                // repeat until starting GC of street segment is starting point
                while (prev != nullptr)
                {
                    route.push_front(*prev); // push prev SS to front
                    totalDistanceTravelled += distanceEarthMiles((*prev).start, (*prev).end); // length of previous SS
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
        if (m_sm->getSegmentsThatStartWith(curr, segs))
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
                    totalDistanceTravelled += distanceEarthMiles((*itr).end, (*itr).start); // distance of current SS
                    
                    // find previous SS
                    StreetSegment* prev = previousSS.find((*itr).start);
                    
                    // repeat until starting GC of street segment is starting point
                    while (prev != nullptr)
                    {
                        route.push_front(*prev); // push prev SS to front
                        totalDistanceTravelled += distanceEarthMiles((*prev).end, (*prev).start); // distance of last SS
                        prev = previousSS.find((*prev).start); // look for starting GC of prev SS
                    }
                    return DELIVERY_SUCCESS;
                }
            }
            curr = points.front();
            points.pop();
        }
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
