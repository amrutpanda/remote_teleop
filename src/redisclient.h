#include <iostream>
#include <string>
#include <memory>
#include <hiredis/hiredis.h>
#include <chrono>
#include <vector>

#define _EIGEN 1

#if _EIGEN == 1
#include <eigen3/Eigen/Dense>
#endif


// custom deleter for deallocating rediscontext object.
struct redisContextDeleter{
    void operator()(redisContext *c) {redisFree(c); }
};


// custom deleter for deallocating redis reply object.
struct redisReplyDeleter{
    void operator()(redisReply *r) {freeReplyObject(r); }
};

enum SUPPORTED_DYTPES 
{
    STR,
    INT,
    DOUBLE,
    EIGEN
} ;

struct containter
{
    std::vector<int> callback_indices;
    std::vector<std::string> keys;
    std::vector<int> dtypes;
    std::vector<void*> objects;
    std::vector<std::pair<int,int>> size_pair;
};



// redis client class.
class RedisClient
{
private:
    enum SUPPORTED_DYTPES 
    {
        STR,
        INT,
        DOUBLE,
        EIGEN
    };

    std::string hostname;
    int port;
    void _connect(std::string hostname, int port);
    
    // container structure usage.
    containter _reads, _writes;
    std::vector< std::pair<int,std::vector<int>> >_group_reads;
    std::vector< std::pair<int,std::vector<int>> > _group_writes;
    
public:

    std::unique_ptr<redisContext> context_;
    void connect();
    void connect(const std::string hostname, int port = 6379);

    std::unique_ptr<redisReply,redisReplyDeleter> reply_;
    
    std::unique_ptr<redisReply, redisReplyDeleter> command(const char* fmt,...);

    std::string get(const std::string& key);
    void set(const std::string& key, std::string& value);
    void set(const std::string& key, const std::string& value);
    void ping();
    bool keyExists(const std::string& key);

    void pipeget(const std::vector<std::string> &keys, std::vector<std::string>&values);
    std::vector<std::string> pipeget(const std::vector<std::string> &keys);
    void pipeset(const std::vector<std::string> &keys, std::vector<std::string>& values);

    int createStringReadCallback( const std::string& key, std::string& object); 
    /**
     * @brief creates a callback for reading int value or array.
     * @param arr_len: size of the array if you want to read a array.
     *                 For a single value arr_len = 1
    */
    int createIntReadCallback( const std::string& key, int* object, int arr_size);
    int createIntReadCallback( const std::string& key, int& object, int arr_size = 1);
    /**
     * @brief creates a callback for reading double value or array.
     * @param arr_len: size of the array if you want to read a array.
     *                 For a single value arr_len = 1
    */
    int createDoubleReadCallback( const std::string& key, double* object, int arr_size);
    int createDoubleReadCallback( const std::string& key, double& object, int arr_size=1);
    
    template< typename _Scalar,int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols >
    int createEigenReadCallback(const std::string& key, Eigen::Matrix<_Scalar,_Rows, _Cols, _Options, _MaxRows, _MaxCols >& object);

    int createStringWriteCallback( const std::string& key, std::string& object);
    /**
     * @brief creates a callback for writing int value or array.
     * @param arr_len: size of the array if you want to write a array.
     *                 For a single value arr_len = 1
    */
    int createIntWriteCallback( const std::string& key, int& object, int arr_size);
    // int createIntWriteCallback( const std::string& key, int& object, int arr_size);
    /**
     * @brief creates a callbac for writing double value or array.
     * @param arr_len: size of the array if you want to write a array.
     *                 For a single value arr_len = 1
    */
    int createDoubleWriteCallback( const std::string& key, double& object, int arr_size);
    // int createDoubleWriteCallback( const std::string& key, double& object, int arr_size);
    // template< typename _Scalar,int rows, int cols, int options, int maxrows, int maxcols >
    // int createEigenWriteCallback(const std::string& key, Eigen::Matrix<_Scalar, rows, cols>* object);

    template< typename _Scalar,int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols >
    int createEigenWriteCallback(const std::string& key, Eigen::Matrix< _Scalar,_Rows, _Cols, _Options,_MaxRows,_MaxCols>& object);

    void createIntGroupReadCallback(int _group_num, const std::string& key, int& object, int arr_size);
    void createIntGroupWriteCallback(int _group_num, const std::string& key, int& object, int arr_size);

    void createDoubleGroupReadCallback(int _group_num, const std::string& key, double* object, int arr_size);
    void createDoubleGroupWriteCallback(int _group_num,const std::string& key, double& object, int arr_size);


    void executeReadCallback(int callback_number);
    void executeWriteCallback( int callback_number);

    void executeAllReadCallbacks();
    void executeAllWriteCallbacks();

    void executeGroupReadCallbacks(int _group_num);
    void executeGroupWriteCallbacks(int _group_num);

    void executeBatchAllReadCallbacks();
    void executeBatchAllWriteCallbacks();

    void executeBatchAllReadWriteCallbacks();

    void StringToIntArray(std::string& str,char delimiter, int* intarr, int arr_len);
    void IntArrayToString(int* intarr, int arr_len, std::string& str, char delimiter);

    void StringToDoubleArray(std::string& str,char delimiter, double* dbarr, int arr_len);
    void DoubleArrayToString(double* dbarr, int arr_len, std::string& str, char delimiter);

    #if _EIGEN == 1

    template< typename _Scalar,int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols >
    void getEigenMatrix(const std::string& key, const Eigen::Matrix< _Scalar,_Rows, _Cols, _Options,_MaxRows,_MaxCols>& matrix);
    
    template< typename _Scalar,int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    void setEigenMatrix(const std::string& key, const Eigen::Matrix< _Scalar,_Rows, _Cols, _Options,_MaxRows,_MaxCols>& matrix);

    template< typename _Scalar,int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    void createEigenGroupReadCallback(int _group_num, const std::string& key,Eigen::Matrix< _Scalar,_Rows, _Cols, _Options,_MaxRows,_MaxCols>& matrix);

    template< typename _Scalar,int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    void createEigenGroupWriteCallback(int _group_num, const std::string& key,Eigen::Matrix< _Scalar,_Rows, _Cols, _Options,_MaxRows,_MaxCols>& matrix);

    #endif
    RedisClient(std::string hostname = "127.0.0.1", int port = 6379);
    // ~RedisClient();
};

// RedisClient::RedisClient(/* args */)
// {
// }

// RedisClient::~RedisClient()
// {
// }

#if _EIGEN == 1

template< typename _Scalar,int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols >
int RedisClient::createEigenReadCallback(const std::string& key, Eigen::Matrix<_Scalar,_Rows, _Cols, _Options,_MaxRows,_MaxCols>& object) {
    int n = _reads.callback_indices.size();

    bool key_found = false;
    if (n != 0)
    {
        for (size_t i = 0; i < n; i++)
        {
            if (_reads.keys[i] == key)
            {
                key_found = true;
            }
            
        }
        
    }

    if (key_found)
        throw std::runtime_error("Key: " + key + " already exists. Please enter an another key.");

    _reads.callback_indices.push_back(n);
    _reads.dtypes.push_back(EIGEN);
    _reads.keys.push_back(key);
    _reads.objects.push_back(object.data());
    _reads.size_pair.push_back(std::make_pair(object.rows(),object.cols()));
    return n;
}

template< typename _Scalar,int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols >
int RedisClient::createEigenWriteCallback(const std::string& key, Eigen::Matrix< _Scalar,_Rows, _Cols, _Options,_MaxRows,_MaxCols>& object) {
    int n = _writes.callback_indices.size();

    bool key_found = false;
    if (n != 0)
    {
        for (size_t i = 0; i < n; i++)
        {
            if (_writes.keys[i] == key)
            {
                key_found = true;
            }
            
        }
        
    }

    if (key_found)
        throw std::runtime_error("Key: " + key + " already exists. Please enter an another key.");

    _writes.callback_indices.push_back(n);
    _writes.dtypes.push_back(EIGEN);
    _writes.keys.push_back(key);
    _writes.objects.push_back(object.data());
    _writes.size_pair.push_back(std::make_pair(object.rows(),object.cols()));
    return n;
}

template< typename _Scalar,int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols >
void RedisClient::getEigenMatrix(const std::string& key, const Eigen::Matrix< _Scalar,_Rows, _Cols, _Options,_MaxRows,_MaxCols>& matrix) {
    int n = _reads.callback_indices.size();
    int key_index = 0;

    bool key_found = false;
    if (n != 0)
    {
        for (size_t i = 0; i < n; i++)
        {
            if (_writes.keys[i] == key)
            {
                key_found = true;
                key_index = i;
            }
            
        }
        
    }

    if (key_found && _reads.dtypes[key_index] != EIGEN)
        throw std::runtime_error("Key: " + key + " already exists but cannot read EIGEN objects. Please enter an another key.");

    std::string str;
    str = get(key);
    int size = matrix.rows() * matrix.cols();
    // StringToDoubleArray(str,',',matrix.data(),size);
    // double* _matrix = const_cast<double*>(matrix.data());
    // StringToDoubleArray(str,',',_matrix,size);
    
    if constexpr(std::is_same<_Scalar,double>::value)
    {
        double* _matrix = const_cast<double*>(matrix.data());
        StringToDoubleArray(str,',',_matrix,size);
    }
    else
    {     
        if constexpr(std::is_same<_Scalar,int>::value)
        {
            int* _matrix = const_cast<int*>(matrix.data());
            StringToIntArray(str,',',_matrix,size);
        }
        else
        {
            throw std::runtime_error("getEigen:: Unknown Scalar type. Only supports double and int");
        }
    }
    
}

template< typename _Scalar,int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols >
void RedisClient::setEigenMatrix(const std::string& key,const Eigen::Matrix< _Scalar,_Rows, _Cols, _Options,_MaxRows,_MaxCols>& matrix) {

    int n = _writes.callback_indices.size();
    int key_index = 0;

    bool key_found = false;
    if (n != 0)
    {
        for (size_t i = 0; i < n; i++)
        {
            if (_writes.keys[i] == key)
            {
                key_found = true;
                key_index = i;
            }
            
        }
        
    }

    if (key_found && _writes.dtypes[key_index] != EIGEN)
        throw std::runtime_error("Key: " + key + " already exists but cannot write to EIGEN objects. Please enter an another key .");

    std::string str;
    int size = matrix.rows() * matrix.cols();
    // DoubleArrayToString(matrix.data(),size,str,',');
    // double* _matrix = const_cast<double*>(matrix.data());
    // DoubleArrayToString(_matrix,size,str,',');

    if constexpr(std::is_same<_Scalar,double>::value)
    {
        double* _matrix = const_cast<double*>(matrix.data());
        DoubleArrayToString(_matrix,size,str,',');
    }
    else
    {
        if constexpr(std::is_same<_Scalar,int>::value)
        {
            int* _matrix = const_cast<int*>(matrix.data());
            IntArrayToString(_matrix,size,str,',');
        }
        else
        {
            throw std::runtime_error("setEigen:: Unknown Scalar type. Only supports double and int");
        }
    }
    
    
    set(key,str);
}


template< typename _Scalar,int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
void RedisClient::createEigenGroupReadCallback(int _group_num, const std::string& key,Eigen::Matrix< _Scalar,_Rows, _Cols, _Options,_MaxRows,_MaxCols>& matrix)
{
    int n = createEigenReadCallback(key,matrix);
    // find the group_num from _group read.
    bool _found = false;
    int index = 0;
    for (int i = 0; i < _group_reads.size(); i++)
    {
        if (_group_reads[i].first == _group_num)
        {
            _found = true;
            index = i;  
        }
    }

    if (!_found)
    {
        // create a new one.
        std::vector<int> _callback_nums;
        _callback_nums.push_back(n);  // save the callback num to group read.
        std::pair<int, std::vector<int> > _pair(_group_num,_callback_nums);
        _group_reads.push_back(_pair);
    }
    else
    {
        // if found. save the callback number to group reads.
        _group_reads[index].second.push_back(n);
    }  
}

template< typename _Scalar,int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
void RedisClient::createEigenGroupWriteCallback(int _group_num, const std::string& key,Eigen::Matrix< _Scalar,_Rows, _Cols, _Options,_MaxRows,_MaxCols>& matrix)
{
    int n = createEigenWriteCallback(key,matrix);
    // find the group_num from _group read.
    bool _found = false;
    int index = 0;
    for (int i = 0; i < _group_writes.size(); i++)
    {
        if (_group_writes[i].first == _group_num)
        {
            _found = true;
            index = i;  
        }
    }

    if (!_found)
    {
        // create a new one.
        std::vector<int> _callback_nums;
        _callback_nums.push_back(n);  // save the callback num to group read.
        std::pair<int, std::vector<int> > _pair(_group_num,_callback_nums);
        _group_writes.push_back(_pair);
    }
    else
    {
        // if found. save the callback number to group reads.
        _group_writes[index].second.push_back(n);
    }  
}
#endif