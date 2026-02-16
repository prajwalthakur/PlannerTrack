
#pragma once

// Implementation of singleton class

template <typename T>
class crSingleton
{
    public:

        // Function to get the instance
        static T*& get()
        {
            static T* singletonInstance = nullptr;
            if(singletonInstance==nullptr)
            {
                singletonInstance = new T();
            }
            return singletonInstance;
        }
        // Delete copy and assignment
        crSingleton(const T&)=delete;
        T& operator=(const T&)=delete;
    protected:
        // Set constructor to protected
        crSingleton() = default;
        ~crSingleton() = default;
};