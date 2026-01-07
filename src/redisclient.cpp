#include "redisclient.h"

RedisClient::RedisClient(std::string hostname, int port) :
    hostname(hostname),port(port) {
        connect();
    }

void RedisClient::_connect(std::string hostname, int port)
{
    // Default redis Server hostname = '127.0.0.1, port = 6379.
    // Connect to new server.
    context_.reset(nullptr);        // set the pointer to null in the beginning.
    redisContext* c;
    c = redisConnect(hostname.c_str(),port);
    // c = redisConnectWithTimeout(hostname.c_str(), port, tv);
    std::unique_ptr<redisContext> context(c);
    if (context == nullptr)
        throw std::runtime_error("Could not allocate context pointer");
    else if (context->err)
        throw std::runtime_error("Error while obtaining context pointer:" + std::string(context->errstr));
    // if everything okay, then move the pointer to the context_.
    context_ = std::move(context);
    
} 

void RedisClient::connect() {
    _connect(hostname,port);
}

std::unique_ptr<redisReply,redisReplyDeleter>  RedisClient::command(const char* fmt,...){
 // send command to the redis server and receive a reply.
    va_list args;
    va_start(args,fmt);
    redisReply *reply = (redisReply*) redisvCommand(context_.get(),fmt,args);
    va_end(args);
    return std::unique_ptr<redisReply, redisReplyDeleter> (reply);
}

void RedisClient::ping() {
    std::unique_ptr<redisReply, redisReplyDeleter> reply;
    reply = command("PING Hello");
    std::cout<< "Pinging to RedisServer: host address = " << context_->tcp.host << " , port = " <<context_->tcp.port << std::endl;
    if (!reply)
        std::cout << "Failed to connect to the server." << std::endl;
    else
        std::cout<< "Ping reply: "<< reply->str << std::endl;
}

bool RedisClient::keyExists(const std::string& key) {
    std::unique_ptr<redisReply, redisReplyDeleter> reply;
    reply = command("EXISTS %s", key.c_str());
    // error checking.
    if (!reply || reply->type == REDIS_REPLY_ERROR || reply->type == REDIS_REPLY_NIL )
        throw std::runtime_error("Error while reading from server with key in EXISTS function " + key );
    if (reply->type != REDIS_REPLY_INTEGER)
        throw std::runtime_error("Redis replied a non-integer reply in EXIST function" + key);
    return (reply->integer == 1);
}

std::string RedisClient::get(const std::string& key) {
    std::unique_ptr< redisReply, redisReplyDeleter> reply;
    reply = command("GET %s",key.c_str());
    if (!reply || reply->type == REDIS_REPLY_ERROR  || reply->type == REDIS_REPLY_NIL)
        throw std::runtime_error("Error while reading from server with key: " + key );
    return reply->str;
}

void RedisClient::set(const std::string& key,std::string& value) {
    std::unique_ptr < redisReply, redisReplyDeleter> reply;
    reply = command("SET %s %s", key.c_str(), value.c_str());
    if (!reply || reply->type == REDIS_REPLY_ERROR )
        throw std::runtime_error("Error while reading from server with key: " + key );
}

void RedisClient::set(const std::string& key, const std::string& value) {
    std::unique_ptr < redisReply, redisReplyDeleter> reply;
    reply = command("SET %s %s", key.c_str(), value.c_str());
    if (!reply || reply->type == REDIS_REPLY_ERROR )
        throw std::runtime_error("Error while reading from server with key: " + key );
}

void RedisClient::pipeset(const std::vector<std::string> &keys, std::vector<std::string>& values) {
    // check the length of keys and value vectors.
    if (keys.size() != values.size())
        std::runtime_error("No. of keys and values are not equal. keys vector length :"+ std::to_string(keys.size()) \
                           + "but the values vector length : " + std::to_string(values.size()));

    for ( size_t i = 0; i < keys.size(); ++i)
    {
        redisAppendCommand(context_.get(), "SET %s %s", keys[i].c_str(),values[i].c_str());
        
    }
    // check whether SET command worked or not.
    redisReply * reply;
    
    for (size_t i = 0; i < keys.size(); ++i)
    {
        if (redisGetReply(context_.get(), (void**) &reply) == REDIS_ERR ) 
            throw std::runtime_error("Error while reading value of the key : " + keys[i]);
            // std::cout<< "Error while executing SET command for the key" << keys[i];
        std::unique_ptr<redisReply, redisReplyDeleter> reply_ptr(reply);

        if (reply_ptr->type == REDIS_REPLY_ERROR)
            throw std::runtime_error("Error while in the reply object for the key : "+ keys[i]);
    }
    
}

//" pipeget function has been overloaded twice. the first function is here." 

std::vector<std::string> RedisClient::pipeget(const std::vector<std::string> &keys) {
    std::vector <std::string> values;
    for (size_t i = 0; i < keys.size(); ++i) {
        redisAppendCommand(context_.get(), "GET %s",keys[i].c_str());
    }
    // Get and store the reply in a vector.
    redisReply *reply;
    int ret;
    for (size_t i = 0; i < keys.size(); ++i)
    {
        if (redisGetReply(context_.get(), (void**) &reply) == REDIS_ERR ) 
            throw std::runtime_error("Error while reading value of the key : " + keys[i]);
            // std::cout<< "Error while executing SET command for the key" << keys[i];
        std::unique_ptr<redisReply, redisReplyDeleter> reply_ptr(reply);
        if (reply_ptr->type == REDIS_REPLY_ERROR || reply_ptr->type == REDIS_REPLY_NIL )
            throw std::runtime_error("Error while in the reply object for the key : "+ keys[i]);
        values.push_back(reply->str);
    }
    return values;
}   

// "pipeget overloaded function 2"
void RedisClient::pipeget(const std::vector<std::string> &keys, std::vector<std::string> &values) {
    for (size_t i = 0; i < keys.size(); i++)
    {
        redisAppendCommand(context_.get(), "GET %s",keys[i].c_str());
    }
    redisReply *reply;
    int ret;
    for (size_t i = 0; i < keys.size(); ++i)
    {
        if (redisGetReply(context_.get(), (void**) &reply) == REDIS_ERR ) 
            throw std::runtime_error("(Overloaded pipeset )Error while reading value of the key : " + keys[i]);
            // std::cout<< "Error while executing SET command for the key" << keys[i];
        std::unique_ptr<redisReply, redisReplyDeleter> reply_ptr(reply);
        if (reply_ptr->type == REDIS_REPLY_ERROR || reply_ptr->type == REDIS_REPLY_NIL )
            throw std::runtime_error("(Overloaded pipeset )Error while in the reply object for the key : "+ keys[i]);
        values.push_back(reply->str);
    }
}

// ****************** Read callbacks ********************

int RedisClient::createStringReadCallback( const std::string& key, std::string& object) {
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
    _reads.dtypes.push_back(STR);
    _reads.keys.push_back(key);
    _reads.objects.push_back(&object);
    _reads.size_pair.push_back(std::make_pair(0,0));
    return n;
}

int RedisClient::createIntReadCallback( const std::string& key, int& object, int arr_size) {
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
    _reads.dtypes.push_back(INT);
    _reads.keys.push_back(key);
    _reads.objects.push_back(&object);
    _reads.size_pair.push_back(std::make_pair(arr_size,0));
    return n;
}

int RedisClient::createDoubleReadCallback( const std::string& key, double& object, int arr_size) {
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
    _reads.dtypes.push_back(DOUBLE);
    _reads.keys.push_back(key);
    _reads.objects.push_back(&object);
    _reads.size_pair.push_back(std::make_pair(arr_size,0));
    return n;
}


void RedisClient::executeReadCallback(int callback_num) {
    int n = _reads.callback_indices.size();

    if (callback_num >= n)
        throw std::runtime_error("Callback number doesn't exist. callback number = "+ std::to_string(callback_num));

    
    // std::vector<std::string> retvals;
    // pipeget(_reads.keys,retvals);
    std::string retval;
    retval =  get(_reads.keys[callback_num]);
    
    switch (_reads.dtypes[callback_num])
    {
    case STR:
        {
            std::string* ptr = (std::string*) _reads.objects[callback_num];
            *ptr = retval;
        }
        break;
    case INT:
        {
            int* ptr = (int*) _reads.objects[callback_num];
            if (_reads.size_pair[callback_num].first != 0 ) 
            {
                // for (int i = 0; i <= _reads.size_pair[callback_num].first; i++)
                // {
                //     *(ptr+i) = std::stoi(retval);
                // }
                StringToIntArray(retval,',',ptr,_reads.size_pair[callback_num].first);
            }
            else
            {
                *ptr = std::stoi(retval);
            }        
        }
        break;
    case DOUBLE:
        {
            double* ptr = (double*) _reads.objects[callback_num];
            if (_reads.size_pair[callback_num].first != 0 ) 
            {
                // for (int i = 0; i <= _reads.size_pair[callback_num].first; i++)
                // {
                //     *(ptr+i) = std::stoi(retval);
                // }
                StringToDoubleArray(retval,',',ptr,_reads.size_pair[callback_num].first);
            }
            else
            {
                *ptr = std::stod(retval);
            }        
        }
    case EIGEN:
        {
            double* ptr = (double*) _reads.objects[callback_num];
            int n = _reads.size_pair[callback_num].first * _reads.size_pair[callback_num].second;
            // double dbarr[n];
            StringToDoubleArray(retval,',',ptr,n);
        }
    
    default:
        break;
    }
    
}


// ****************** Write callbacks ********************
int RedisClient::createStringWriteCallback( const std::string& key, std::string& object) {
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
    _writes.dtypes.push_back(STR);
    _writes.keys.push_back(key);
    _writes.objects.push_back(&object);
    _writes.size_pair.push_back(std::make_pair(0,0));
    return n;
}

int RedisClient::createIntWriteCallback( const std::string& key, int& object, int arr_size) {
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
    _writes.dtypes.push_back(INT);
    _writes.keys.push_back(key);
    _writes.objects.push_back(&object);
    _writes.size_pair.push_back(std::make_pair(arr_size,0));
    return n;
}

int RedisClient::createDoubleWriteCallback( const std::string& key, double& object, int arr_size) {
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
    _writes.dtypes.push_back(DOUBLE);
    _writes.keys.push_back(key);
    _writes.objects.push_back(&object);
    _writes.size_pair.push_back(std::make_pair(arr_size,0));
    return n;
}

void RedisClient::executeWriteCallback(int callback_num) {
    int n = _writes.callback_indices.size();
    if (callback_num >= n)
        throw std::runtime_error("write Callback number" + std::to_string(callback_num) + "is invalid.");

    std::string str;
    switch (_writes.dtypes[callback_num])
    {
    case STR:
        {
            std::string* ptr = (std::string*)_writes.objects[callback_num];
        }
        break;
    case INT:
        {
            int* ptr = (int*) _writes.objects[callback_num];
            if (_writes.size_pair[callback_num].first != 0)
            {
                IntArrayToString(ptr,_writes.size_pair[callback_num].first,str,',');
            }
            else
            {
                str = std::to_string(*ptr);
            }
        }
        break;
    case DOUBLE:
        {
            double* ptr = (double*) _writes.objects[callback_num];
            if (_writes.size_pair[callback_num].first != 0)
            {
                DoubleArrayToString(ptr,_writes.size_pair[callback_num].first,str,',');
            }
            else
            {
                str = std::to_string(*ptr); 
            }
        }
    case EIGEN:
        {
            double* ptr = (double*) _writes.objects[callback_num];
            int n = _writes.size_pair[callback_num].first * _writes.size_pair[callback_num].second;
            DoubleArrayToString(ptr,n,str,',');
        }
    default:
        break;
    }

    set(_writes.keys[callback_num],str);
}



// ********************** data type converters **********************

void RedisClient::StringToIntArray(std::string& str, char delimiter, int* intarr, int arr_len) {
    std::string tstr = "";
    int* ptr = intarr;
    // char delimiter = '.';
    int len = 0;
    for (size_t i = 0; i < str.size(); i++)
    {
        if(str[i] == delimiter )
        {
            *ptr = std::stoi(tstr);
            len += 1;
            ptr++;
            tstr = "";
        }
        else   
            tstr += str[i];
    }

    *ptr = std::stoi(tstr);
    len += 1;

    if (len != arr_len )
        throw std::runtime_error("Number of elements to convert to integer doesn't match the given array length. " \
                                   + std::to_string(len)+" != " + std::to_string(arr_len));    
}

void RedisClient::IntArrayToString(int* intarr, int arr_len, std::string& str, char delimiter) {
    std::string tstr = "";
    int* ptr = intarr;
    for (size_t i = 0; i < arr_len; i++)
    {
        tstr = tstr + std::to_string(*ptr) + delimiter;
        ptr++;
    }
    tstr.erase(tstr.end() -1);
    str = tstr;
    
}

void RedisClient::StringToDoubleArray(std::string& str, char delimiter, double* dbarr, int arr_len) {
    std::string tstr = "";
    double* ptr = dbarr;
    // char delimiter = '.';
    // std::cout << "dbarr: " << str << std::endl;
    int len = 0;
    for (size_t i = 0; i < str.size(); i++)
    {
        if(str[i] == delimiter )
        {
            *ptr = std::stod(tstr);
            len += 1;
            ptr++;
            tstr = "";
        }
        else   
            tstr += str[i];
    }

    *ptr = std::stod(tstr);
    len += 1;
    
    if (len != arr_len )
        throw std::runtime_error("Number of elements to convert to double doesn't match the given array length. " + std::to_string(len) + " != " + std::to_string(arr_len));    
}

void RedisClient::DoubleArrayToString(double* dbarr, int arr_len, std::string& str, char delimiter) {
    std::string tstr = "";
    double* ptr = dbarr;
    for (size_t i = 0; i < arr_len; i++)
    {
        tstr = tstr + std::to_string(*ptr) + delimiter;
        ptr++;
    }
    tstr.erase(tstr.end() -1);
    str = tstr;
    
}

// ********************* Execute all callbacks ***********************

void RedisClient::executeAllReadCallbacks() {
    int n = _reads.callback_indices.size();
    for (int callback_num = 0; callback_num < n; callback_num++)
    {
        std::string retval;
        retval =  get(_reads.keys[callback_num]);
        switch (_reads.dtypes[callback_num])
        {
        case STR:
            {
                std::string* ptr = (std::string*) _reads.objects[callback_num];
                *ptr = retval;
            }
            break;
        case INT:
            {
                int* ptr = (int*) _reads.objects[callback_num];
                if (_reads.size_pair[callback_num].first != 0 ) 
                {
                    // for (int i = 0; i <= _reads.size_pair[callback_num].first; i++)
                    // {
                    //     *(ptr+i) = std::stoi(retval);
                    // }
                    StringToIntArray(retval,',',ptr,_reads.size_pair[callback_num].first);
                }
                else
                {
                    *ptr = std::stoi(retval);
                }        
            }
            break;
        case DOUBLE:
            {
                double* ptr = (double*) _reads.objects[callback_num];
                if (_reads.size_pair[callback_num].first != 0 ) 
                {
                    // for (int i = 0; i <= _reads.size_pair[callback_num].first; i++)
                    // {
                    //     *(ptr+i) = std::stoi(retval);
                    // }
                    StringToDoubleArray(retval,',',ptr,_reads.size_pair[callback_num].first);
                }
                else
                {
                    *ptr = std::stod(retval);
                }
            }
            break; 
        case EIGEN:
            {
                double* ptr = (double*) _reads.objects[callback_num];
                int n = _reads.size_pair[callback_num].first * _reads.size_pair[callback_num].second;
                // double dbarr[n];
                StringToDoubleArray(retval,',',ptr,n);
            }
            break;
        
        default:
            break;
        }
    }
    
}

void RedisClient::executeAllWriteCallbacks() {
    int n = _writes.callback_indices.size();

    for (int callback_num = 0; callback_num < n; callback_num++)
    {
        std::string str;
        switch (_writes.dtypes[callback_num])
        {
        case STR:
            {
                std::string* ptr = (std::string*)_writes.objects[callback_num];
            }
            break;
        case INT:
            {
                int* ptr = (int*) _writes.objects[callback_num];
                if (_writes.size_pair[callback_num].first != 0)
                {
                    IntArrayToString(ptr,_writes.size_pair[callback_num].first,str,',');
                }
                else
                {
                    str = std::to_string(*ptr);
                }
            }
            break;
        case DOUBLE:
            {
                double* ptr = (double*) _writes.objects[callback_num];
                if (_writes.size_pair[callback_num].first != 0)
                {
                    DoubleArrayToString(ptr,_writes.size_pair[callback_num].first,str,',');
                }
                else
                {
                    str = std::to_string(*ptr); 
                }
            }
            break;
        case EIGEN:
            {
                double* ptr = (double*) _writes.objects[callback_num];
                int n = _writes.size_pair[callback_num].first * _writes.size_pair[callback_num].second;
                DoubleArrayToString(ptr,n,str,',');
            }
            break;
        default:
            break;
        }

        set(_writes.keys[callback_num],str);
    }
    
}

//*********************** Group callbacks. ************** */

void RedisClient::executeGroupReadCallbacks(int _group_num)
{
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
        throw std::runtime_error("Inside groupReadCallback. Invalid group number: " + _group_num );
    else
    {
        for (int i = 0; i < _group_reads[index].second.size(); i++)
        {
            executeReadCallback(_group_reads[index].second[i]);
        }
        
    }
    
}

void RedisClient::executeGroupWriteCallbacks(int _group_num)
{
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
        throw std::runtime_error("Inside groupReadCallback. Invalid group number: " + _group_num );
    else
    {
        for (int i = 0; i < _group_writes[index].second.size(); i++)
        {
            executeReadCallback(_group_writes[index].second[i]);
        }
        
    }
    
}

void RedisClient::createDoubleGroupReadCallback(int _group_num, const std::string& key, double& object, int arr_size)
{
    int n = createDoubleReadCallback(key,object,arr_size);
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

void RedisClient::createDoubleGroupWriteCallback(int _group_num, const std::string& key, double& object, int arr_size)
{
    int n = createDoubleWriteCallback(key,object,arr_size);
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

void RedisClient::createIntGroupReadCallback(int _group_num, const std::string& key, int& object, int arr_size)
{
    int n = createIntReadCallback(key,object,arr_size);
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

void RedisClient::createIntGroupWriteCallback(int _group_num, const std::string& key, int& object, int arr_size)
{
    int n = createIntWriteCallback(key,object,arr_size);
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
