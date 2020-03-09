//
//  ExpandableHashMap.h
//  proj4
//
//  Created by Alex Chen on 3/5/20.
//  Copyright © 2020 cs. All rights reserved.
//

#ifndef EXPANDABLE_HASH_MAP
#define EXPANDABLE_HASH_MAP

// vector of linked lists of Pairs, containing KeyType and ValueType
#include <vector>
#include <list>

template<typename KeyType, typename ValueType>
class ExpandableHashMap
{
public:
    ExpandableHashMap(double maximumLoadFactor = 0.5);
    ~ExpandableHashMap();
    void reset();
    int size() const;
    void associate(const KeyType& key, const ValueType& value);

      // for a map that can't be modified, return a pointer to const ValueType
    const ValueType* find(const KeyType& key) const;

      // for a modifiable map, return a pointer to modifiable ValueType
    ValueType* find(const KeyType& key)
    {
        return const_cast<ValueType*>(const_cast<const ExpandableHashMap*>(this)->find(key));
    }

      // C++11 syntax for preventing copying and assignment
    ExpandableHashMap(const ExpandableHashMap&) = delete;
    ExpandableHashMap& operator=(const ExpandableHashMap&) = delete;

private:
    
    struct Pair
    {
        KeyType m_key;
        ValueType m_value;
    };
    
    void clearMap();
    
    std::vector<std::list<Pair*>> m_hashTable; // hash table is a vector containing a list of Pairs of GeoCoords and vector of street segments
    int m_buckets; // number of buckets
    int m_associations; // each bucket will have a certain number of associations (num of Pairs in linked list)
    double m_maxLoad; // max load of hash map
    
    unsigned int getHashResult(const KeyType& key, int buckets) const; // return hash result, before performing modulus
};

template<typename KeyType, typename ValueType>
ExpandableHashMap<KeyType, ValueType>::ExpandableHashMap(double maximumLoadFactor)
{
    m_hashTable.resize(8);
    m_buckets = 8; // initialize size of hash table
    m_associations = 0; // number of key value pairs
    if (maximumLoadFactor <= 0) // initialize max load of hash table
        m_maxLoad = 0.5;
    else
        m_maxLoad = maximumLoadFactor;
}

template<typename KeyType, typename ValueType>
ExpandableHashMap<KeyType, ValueType>::~ExpandableHashMap()
{
    clearMap();
}

template<typename KeyType, typename ValueType>
void ExpandableHashMap<KeyType, ValueType>::reset()
{
    clearMap();
    m_hashTable.resize(8);
    m_buckets = 8;
    m_associations = 0;
}

template<typename KeyType, typename ValueType>
int ExpandableHashMap<KeyType, ValueType>::size() const
{
    return m_associations;
}

template<typename KeyType, typename ValueType>
void ExpandableHashMap<KeyType, ValueType>::associate(const KeyType& key, const ValueType& value)
{
    if (((m_associations+1) / m_buckets) > m_maxLoad)
    {
        std::vector<std::list<Pair*>> m_newHashTable; // new hash table
        m_newHashTable.resize(2*m_buckets);
        
        // rehash all linked lists inside existing buckets and add to new container
        for (int i=0; i<m_buckets; i++)
        {
            for (auto itr = m_hashTable[i].begin(); itr != m_hashTable[i].end(); itr++)
            {
                unsigned int h = getHashResult((*itr)->m_key, 2*m_buckets);
                m_newHashTable[h].push_back((*itr));
            }
        }
        
        m_buckets *= 2; // double number of buckets
        m_hashTable.resize(m_buckets); // resize original hash map
        
        m_hashTable = m_newHashTable; // shallow copy all lists and Pair pointers back into original hash map
        
        // loop through temp hash map and assign Pair pointers to nullptr
        for (auto itr = m_newHashTable.begin(); itr != m_newHashTable.end(); itr++)
        {
            for (auto itr2 = (*itr).begin(); itr2 != (*itr).end(); itr2++)
                (*itr2) = nullptr;
        }
    }
    ValueType* result = find(key);
    
    // if key is not found, add new Pair pointer to linked list in appropriate bucket
    if (result == nullptr)
    {
        unsigned int h = getHashResult(key, m_buckets);
        m_hashTable[h].push_back(new Pair{key, value});
        m_associations++;
        return;
    }
    *result = value;
}

template<typename KeyType, typename ValueType>
const ValueType* ExpandableHashMap<KeyType, ValueType>::find(const KeyType& key) const
{
    unsigned int h = getHashResult(key, m_buckets);
    for (auto itr = m_hashTable[h].begin(); itr != m_hashTable[h].end(); itr++)
    {
        // if key is found, return address of m_value
        if ((*itr)->m_key == key)
            return &(*itr)->m_value;
    }
    return nullptr; // return nullptr if key not found
}

template<typename KeyType, typename ValueType>
void ExpandableHashMap<KeyType, ValueType>::clearMap()
{
    for (auto itr = m_hashTable.begin(); itr != m_hashTable.end(); )
    {
        for (auto itr2 = (*itr).begin(); itr2 != (*itr).end(); )
        {
            delete *itr2;
            itr2 = (*itr).erase(itr2);
        }
        itr = m_hashTable.erase(itr);
    }
}

template<typename KeyType, typename ValueType>
unsigned int ExpandableHashMap<KeyType, ValueType>::getHashResult(const KeyType& key, int buckets) const
{
    unsigned int hasher(const KeyType& k); // prototype
    unsigned int h = hasher(key); // hash key and get bucket number
    return h % buckets;
}

#endif /* EcpandableHashMap.h */
