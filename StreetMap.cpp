#include "provided.h"
#include "ExpandableHashMap.h"
#include <fstream>
#include <string>
#include <vector>
#include <functional>
using namespace std;

unsigned int hasher(const GeoCoord& g)
{
    return hash<string>()(g.latitudeText + g.longitudeText);
}

class StreetMapImpl
{
public:
    StreetMapImpl();
    ~StreetMapImpl();
    bool load(string mapFile);
    bool getSegmentsThatStartWith(const GeoCoord& gc, vector<StreetSegment>& segs) const;
private:
    ExpandableHashMap<GeoCoord, vector<StreetSegment>>* m_hashMap;
};

StreetMapImpl::StreetMapImpl()
{
}

StreetMapImpl::~StreetMapImpl()
{
//    m_hashMap->clearMap();
}

bool StreetMapImpl::load(string mapFile)
{
    // read in map data
    ifstream infile("mapdata.txt");
    if (!infile)
        return false;
    string s;
    
    // read line containing street until the end
    while (getline(infile, s))
    {
        string street = s; // first line
        int k; // second line containing num of following lines
        infile >> k;
        infile.ignore(10000, '\n');
        
        // for consequent lines containing StreetSegments
        for (int i=0; i<k; i++)
        {
            bool first = true;
            string line;
            getline(infile, line);
            int j=0;
            GeoCoord gc1;
            GeoCoord gc2;
            
            // each line contains two GeoCoords
            for (int z=0; z<2; z++)
            {
                string lat = "", lon = "";
                while (j<line.length() && j != ' ')
                {
                    lat += line[j];
                    j++;
                }
                j++;
                while (j<line.length() && j != ' ')
                {
                    lon += line[j];
                    j++;
                }
                j++;
                GeoCoord gc_temp(lat, lon);
                if (first)
                {
                    // gc1 now contains lat and lon based on first two coords
                    gc1 = gc_temp;
                    first = false;
                }
                else
                    // gc2 now contains lat and lon based on last two coords
                    gc2 = gc_temp;
            }
            
            // form two street segments based on start and end GeoCoords and the reverses
            StreetSegment ss1(gc1, gc2, street);
            StreetSegment ss2(gc2, gc1, street);
            vector<StreetSegment>* result1 = m_hashMap->find(gc1);
            vector<StreetSegment>* result2 = m_hashMap->find(gc2);
            
            // if result1 is nullptr, then key is not found
            if (result1 == nullptr)
                m_hashMap->associate(gc1, vector<StreetSegment> {ss1}); // insert vector with that street segment
            else
                result1->push_back(ss1); // take vector value and insert street segment
            if (result2 == nullptr)
                m_hashMap->associate(gc2, vector<StreetSegment> {ss2});
            else
                result2->push_back(ss2);
        }
    }
    return true;
}

bool StreetMapImpl::getSegmentsThatStartWith(const GeoCoord& gc, vector<StreetSegment>& segs) const
{
    vector<StreetSegment>* result = m_hashMap->find(gc);
    if (result == nullptr)
        return false;
    segs = *result;
    return true;
}

//******************** StreetMap functions ************************************

// These functions simply delegate to StreetMapImpl's functions.
// You probably don't want to change any of this code.

StreetMap::StreetMap()
{
    m_impl = new StreetMapImpl;
}

StreetMap::~StreetMap()
{
    delete m_impl;
}

bool StreetMap::load(string mapFile)
{
    return m_impl->load(mapFile);
}

bool StreetMap::getSegmentsThatStartWith(const GeoCoord& gc, vector<StreetSegment>& segs) const
{
   return m_impl->getSegmentsThatStartWith(gc, segs);
}
