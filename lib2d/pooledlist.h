
#ifndef _POOLEDLIST_
#define _POOLEDLIST_

#include <list>

#include <stdio.h>

using namespace std;

template<typename T>
struct TNode {inline TNode(){} T datum; class TNode* next; class TNode* prev;};


template<typename T, int blockSize>
class TBlock {
public:
    typedef TNode<T> Node;
    
    Node N[blockSize];
    
    inline explicit TBlock() {}
    inline ~TBlock() {}
    
    inline explicit TBlock(Node* firstPrev, Node* lastNext)
    {
        for(int i=0; i<blockSize; i++)
        {
            N[i].prev = N+i-1;
            N[i].next = N[i].prev+2;
        }
        N[0].prev = firstPrev;
        N[blockSize-1].next = lastNext;
    }
};



template<typename T, int blockSize>
class Pool {
public:
    typedef TNode<T> Node;
    typedef TBlock<T,blockSize> Block;
    
    list<Block*> L;
    Node usedSentinel, availableSentinel;
    
    inline explicit Pool()
    {
        availableSentinel.next = availableSentinel.prev = &availableSentinel;
        usedSentinel.next = usedSentinel.prev = &usedSentinel;
    }
    
    Pool(const class Pool& inp)
    {
        printf( "copying a pool is bad\n" );
        assert(false);
    }
    
    inline ~Pool()
    {
        for( typename list<TBlock<T,blockSize>*>::iterator itr = L.begin(); itr!=L.end(); itr++ )
            delete(*itr);
    }
    
    inline T* newPointer()
    {   
        if( availableSentinel.next == availableSentinel.prev ) expand();
        
        Node* r = availableSentinel.next;
        r->prev->next = r->next;
        r->next->prev = r->prev;
        r->next = usedSentinel.next;
        r->prev = &usedSentinel;
        usedSentinel.next->prev = r;
        usedSentinel.next = r;
        
        return (T*)r;
    }
    
    inline void freePointer(T* t)
    {
        Node* r = (Node*)t;
        r->prev->next = r->next;
        r->next->prev = r->prev;
        r->next = availableSentinel.next;
        r->prev = &availableSentinel;
        availableSentinel.next->prev = r;
        availableSentinel.next = r;
    }
    
    void clear()
    {
        //for( typename list<Block*>::iterator itr = L.begin(); itr!=L.end(); itr++) delete(*itr);
        //L.clear();
        usedSentinel.prev->next = &availableSentinel;
        usedSentinel.next->prev = availableSentinel.prev;
        availableSentinel.prev->next = usedSentinel.next;
        availableSentinel.prev = usedSentinel.prev;
        
        usedSentinel.next = usedSentinel.prev = &usedSentinel;
    }
    
    inline void expand()
    {   
        L.push_back( new Block(&availableSentinel, &availableSentinel) );
        availableSentinel.next = (*(L.rbegin()))->N;
        availableSentinel.prev = (*(L.rbegin()))->N+(blockSize-1);
    }
    
    void sanityCheck()
    {
        printf( "availableSentinel: %x %x\n", availableSentinel.next, availableSentinel.prev );
        for( Node* t = availableSentinel.next; t!=&availableSentinel; t=t->next )
        {
            printf( "node: %x next: %x prev: %x\n", t, t->next, t->prev );
        }
        
        printf( "usedSentinel: %x %x\n", usedSentinel.next, usedSentinel.prev );
        for( Node* t = usedSentinel.next; t!=&usedSentinel; t=t->next )
        {
            printf( "node: %x next: %x prev: %x\n", t, t->next, t->prev );
        }
    }
};


template<typename T, int blocksize = 32>
class PooledList {
private:
    int list_size;
    
public:
    typedef TNode<T> Node;
    typedef Pool<Node, blocksize> pooltype;
    Node sentinel;
    
    Pool<Node, blocksize>* P;
    
    inline explicit PooledList( Pool<Node, blocksize>& inP ) : list_size(0)
    {
        P = &inP;
        sentinel.next = sentinel.prev = &sentinel;
    }
    
    inline PooledList(const PooledList& inL)
    {
        list_size = inL.list_size;
        sentinel.next = inL.sentinel.next;
        sentinel.prev = inL.sentinel.prev;
    }
    
    inline PooledList& operator=(const PooledList& inL)
    {
        list_size = inL.list_size;
        sentinel.next = inL.sentinel.next;
        sentinel.prev = inL.sentinel.prev;
        return *this;
    }
    
    
    
    Node* add(const T& t)
    {
        Node* n = P->newPointer();
        *((T*)(n)) = t; //T's assignment operator
        
        n->prev = sentinel.prev;
        n->next = &sentinel;
        sentinel.prev->next = n;
        sentinel.prev = n;
        
        list_size++;
        
        return n;
    }
    
    void erase(Node* n)
    {
        n->next->prev = n->prev;
        n->prev->next = n->next;
        P->freePointer(n);
        
        list_size--;
    }
    
    inline Node* begin()
    {
        return sentinel.next;
    }
        
    inline Node* end()
    {
        return sentinel.prev->next;
    }
    
    inline int size()
    {
        return list_size;
    }
};


#endif

