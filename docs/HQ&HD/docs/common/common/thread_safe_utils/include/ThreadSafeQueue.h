#ifndef THREAD_SAFE_QUEUE_H
#define THREAD_SAFE_QUEUE_H

#include <iostream>
#include <string>
#include <condition_variable>
#include <mutex>
#include <stack>
#include <vector>
#include <memory>

template<class T>
struct ListNode
{
   double timeStamp;
   ListNode* pre;
   ListNode* next;
   T value;
};

template<class T>
class ThreadSafeQueue
{

public:
    ThreadSafeQueue(){};
    ThreadSafeQueue(int size)
    {
      header = new ListNode<T>();
      ListNode<T>* ptr = header;
      ptr->timeStamp = 0;
      for(int i = 0 ; i < size ; i ++)
      {
        ptr->next = new ListNode<T>();
        ptr->next->pre = ptr;
        ptr = ptr->next;      
        ptr->timeStamp = 0;
      }
      ptr->next = header;
      header->pre = ptr;
    }
    ~ThreadSafeQueue()
    {
      ListNode<T>* ptr = header->next;
      ListNode<T>* tmp = header->next;
      while (ptr != header)
      {
        tmp = ptr;
        ptr = ptr->next;
        delete tmp;
      }
      delete header;
      header = NULL;

    }

    void ClearQueue() 
    {
      ListNode<T>* ptr = header->next;
      while (ptr != header)
      {
        ptr->timeStamp = 0;
        ptr->value = {};
        ptr = ptr->next;
      }
    }

    //增
    void Push(const T& t, double time_stamp)
    {
       header = header->pre;
       header->timeStamp = time_stamp;
       header->value = t;
    }

    void Push(T& t, double time_stamp)
    {
      header = header->pre;
      header->timeStamp = time_stamp;
      header->value = t;
    }

    void Push(T& t)
    {
      header = header->pre;
      header->value = t;
    }

    void Push(const T& t)
    {
      header = header->pre;
      header->value = t;
    }
  
    //查
    bool TryPop(T& t) 
    {
       if (header->timeStamp == 0)
       {
         return false;
       }
       t = header->value;
       return true;
    }

    bool Pop(T& t) 
    {
       t = header->value;
       return true;
    }

    bool PopAll(std::vector<T> t)
    {
      ListNode<T>* ptr = header->next;
      t.emplace_back(header->value);
      while (ptr != header)
      {
        t.emplace_back(ptr->value);
        ptr = ptr->next;
      }
      return true;
    }

    bool CheckTimeStamp(T& t, double time_stamp) 
    {
       ListNode<T>* ptr = header->next;
       while(ptr != header)
       {
         if (ptr->timeStamp <= time_stamp && time_stamp < ptr->pre->timeStamp)
         {
           break;
         }
         ptr = ptr->next;
       }
       if (ptr != header)
       {
         t = ptr->value;
         return true;
       }else
       {
         return false;
       }
    }


private:
    ThreadSafeQueue(const ThreadSafeQueue&) = delete;
    ThreadSafeQueue& operator=(const ThreadSafeQueue&) = delete;
    ThreadSafeQueue(ThreadSafeQueue&&) = delete;
    ThreadSafeQueue& operator=(ThreadSafeQueue&&) = delete;

private:
    ListNode<T>* header = NULL;
};

#endif