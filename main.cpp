#include "provided.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "ExpandableHashMap.h"
using namespace std;

bool loadDeliveryRequests(string deliveriesFile, GeoCoord& depot, vector<DeliveryRequest>& v);
bool parseDelivery(string line, string& lat, string& lon, string& item);

//int main(int argc, char *argv[])
//{
//    if (argc != 3)
//    {
//        cout << "Usage: " << argv[0] << " mapdata.txt deliveries.txt" << endl;
//        return 1;
//    }
//
//    StreetMap sm;
//
//    if (!sm.load(argv[1]))
//    {
//        cout << "Unable to load map data file " << argv[1] << endl;
//        return 1;
//    }
//
//    GeoCoord depot;
//    vector<DeliveryRequest> deliveries;
//    if (!loadDeliveryRequests(argv[2], depot, deliveries))
//    {
//        cout << "Unable to load delivery request file " << argv[2] << endl;
//        return 1;
//    }
//
//    cout << "Generating route...\n\n";
//
//    DeliveryPlanner dp(&sm);
//    vector<DeliveryCommand> dcs;
//    double totalMiles;
//    DeliveryResult result = dp.generateDeliveryPlan(depot, deliveries, dcs, totalMiles);
//    if (result == BAD_COORD)
//    {
//        cout << "One or more depot or delivery coordinates are invalid." << endl;
//        return 1;
//    }
//    if (result == NO_ROUTE)
//    {
//        cout << "No route can be found to deliver all items." << endl;
//        return 1;
//    }
//    cout << "Starting at the depot...\n";
//    for (const auto& dc : dcs)
//        cout << dc.description() << endl;
//    cout << "You are back at the depot and your deliveries are done!\n";
//    cout.setf(ios::fixed);
//    cout.precision(2);
//    cout << totalMiles << " miles travelled for all deliveries." << endl;
//}
//
//bool loadDeliveryRequests(string deliveriesFile, GeoCoord& depot, vector<DeliveryRequest>& v)
//{
//    ifstream inf(deliveriesFile);
//    if (!inf)
//        return false;
//    string lat;
//    string lon;
//    inf >> lat >> lon;
//    inf.ignore(10000, '\n');
//    depot = GeoCoord(lat, lon);
//    string line;
//    while (getline(inf, line))
//    {
//        string item;
//        if (parseDelivery(line, lat, lon, item))
//            v.push_back(DeliveryRequest(item, GeoCoord(lat, lon)));
//    }
//    return true;
//}
//
//bool parseDelivery(string line, string& lat, string& lon, string& item)
//{
//    const size_t colon = line.find(':');
//    if (colon == string::npos)
//    {
//        cout << "Missing colon in deliveries file line: " << line << endl;
//        return false;
//    }
//    istringstream iss(line.substr(0, colon));
//    if (!(iss >> lat >> lon))
//    {
//        cout << "Bad format in deliveries file line: " << line << endl;
//        return false;
//    }
//    item = line.substr(colon + 1);
//    if (item.empty())
//    {
//        cout << "Missing item in deliveries file line: " << line << endl;
//        return false;
//    }
//    return true;
//}

int main()
{
// Define a hashmap that maps strings to doubles and has a maximum
// load factor of 0.3. It will initially have 8 buckets when empty.
ExpandableHashMap<string,double> nameToGPA(0.3);
 // Add new items to the hashmap. Inserting the third item will cause
// the hashmap to increase the number of buckets (since the maximum
// load factor is 0.3), forcing a rehash of all items.
nameToGPA.associate("Carey", 3.5); // Carey has a 3.5 GPA
nameToGPA.associate("David", 3.99); // David beat Carey
nameToGPA.associate("Abe", 3.2); // Abe has a 3.2 GPA

double* davidsGPA = nameToGPA.find("David");
if (davidsGPA != nullptr)
 *davidsGPA = 1.5; // after a re-grade of David’s exam
 nameToGPA.associate("Carey", 4.0); // Carey deserves a 4.0
 // replaces old 3.5 GPA
double* lindasGPA = nameToGPA.find("Linda");
if (lindasGPA == nullptr)
 cout << "Linda is not in the roster!" << endl;
else
 cout << "Linda’s GPA is: " << *lindasGPA << endl;
}
