// #ifdef _LOOPTIMER_H
// #define _LOOPTIMER_H
#include <iostream>
#include <chrono>
// #include <signal.h>
#include <ctime>
// #include <unistd.h>
#include <thread>

bool _loop_exit = true;

void _loop_sig_handler(int signum)
{
    _loop_exit = false;
    std::cout << "Exiting Looptimer\n";
}


 // set ctime
//  time_t now = time(0);
//  tm* timeinfo = localtime(&now);
//  strftime(timestamp,sizeof(timestamp), "%Y-%m-%d %H:%M:%S", timeinfo);
//  first_loop = false;
//  counter++;

class LoopTimer
{
private:
    bool _print_warning = false;
    bool _overtime_detected = false;
    double _loop_frequency = 100;
    std::chrono::nanoseconds _ns_update_interval;
    long long counter = 0;
    long long _overtime_loop_counter = 0;
    std::chrono::high_resolution_clock::time_point _loop_start_time;
    // variable for average loop frequency;
    float _avg_loop_frequency = 0.0;
    std::chrono::high_resolution_clock::time_point _current_loop_start_time;
    std::chrono::high_resolution_clock::time_point _next_loop_start_time;
    std::chrono::high_resolution_clock::time_point _prev_loop_start_time;
    double _loop_wait_time;
    double _avg_wait_time;
    double _loop_completion_time;

    std::chrono::nanoseconds _loop_overtime_interval;
    double _average_overtime_ms = 0.0;

public:
    LoopTimer(/* args */)
    {
        // signal(SIGINT,_loop_sig_handler);
        // set default frequency interval;
        _ns_update_interval = std::chrono::nanoseconds(static_cast<unsigned int>(1e9/_loop_frequency));

    };
    void setLoopFrequency(double _hz) 
    {
        _loop_frequency = _hz;
        _ns_update_interval = std::chrono::nanoseconds(static_cast<unsigned int>(1e9 / _hz));
        _loop_overtime_interval = _ns_update_interval + 
                                  std::chrono::nanoseconds(static_cast<unsigned int>(100*1e6)); // 100 miliseconds.
    };

    void InitializeTimer(unsigned int _initial_wait_miliseconds = 0.1)
    {
        _current_loop_start_time = std::chrono::high_resolution_clock::now();
        _prev_loop_start_time = _current_loop_start_time;
        _loop_start_time = _current_loop_start_time;
        auto _ns_initial_wait = std::chrono::nanoseconds(static_cast<unsigned int>(_initial_wait_miliseconds*1e6));
        _next_loop_start_time = _current_loop_start_time + _ns_initial_wait;

        counter = 0;
        _overtime_loop_counter = 0.0;
        _average_overtime_ms = 0.0;
    }

    void setOverTime(double _tms) // in miliseconds.
    {
        _loop_overtime_interval = _ns_update_interval + std::chrono::nanoseconds(static_cast<unsigned int>(_tms * 1e6));
    }

    void setPercentageOvetimeAllowed( double _ps, bool print_warning = false)
    {
        _print_warning = print_warning;
        _loop_overtime_interval = std::chrono::nanoseconds(static_cast<unsigned int>(((1 + 0.01* _ps)* 1e9)));
        double _time = (1 + _ps*0.01)/_loop_frequency;
        _loop_overtime_interval = std::chrono::nanoseconds(static_cast<unsigned int>(_time* 1e9));
    }

    bool WaitForNextLoop()
    {
        // set the current time stamp;
        counter++;
        bool retval = true;
        _current_loop_start_time = std::chrono::high_resolution_clock::now();
        _loop_completion_time = std::chrono::duration<double>(_current_loop_start_time - _prev_loop_start_time).count() * 1e3;
        _loop_wait_time = std::chrono::duration<double>(_next_loop_start_time - _current_loop_start_time).count() * 1e3; // in milliseconds.
        _avg_wait_time += (_loop_wait_time - _avg_wait_time)/counter;

        if (_current_loop_start_time < _next_loop_start_time)
        {
            // std::cout << "Thread sleep\n";
            _current_loop_start_time = std::chrono::high_resolution_clock::now();
            std::this_thread::sleep_for(_next_loop_start_time - _current_loop_start_time);
            _next_loop_start_time +=  _ns_update_interval;
            _prev_loop_start_time = std::chrono::high_resolution_clock::now(); 
            // std::cout << "Thread active\n";
        }
        else
        {
            std::chrono::duration _dt = std::chrono::duration_cast<std::chrono::nanoseconds>
                                        (_current_loop_start_time - _next_loop_start_time);
            // std::cout << _dt.count() << " " << _loop_overtime_interval.count() << std::endl;
            if ( _dt < _loop_overtime_interval)
            {
                if (_print_warning)
                    std::cout << "Overtime detected.But less than threshold. Continuing the loop." << std::endl;

                ++_overtime_loop_counter;
                double _ot_wait_time = std::chrono::duration<double>(_dt).count() * 1e3;
                _average_overtime_ms += (_ot_wait_time - _average_overtime_ms)/_overtime_loop_counter;
            }
            else
            {
                std::cout << "Loop Overtime beyond the threshold value: " << _dt.count() << std::endl;
                std::cout << "Stopping Loop execution" << std::endl;
                return false;
            }
            _current_loop_start_time = std::chrono::high_resolution_clock::now();
            _next_loop_start_time = _current_loop_start_time + _ns_update_interval;
            _prev_loop_start_time = _current_loop_start_time;
        }   

        return retval;
    };

    unsigned long long elapsedCycles()
    {
        return counter;
    };

    double elapsedTime()
    {
        return std::chrono::duration<double>(_next_loop_start_time - _loop_start_time).count();
    }

    void printTimerHistory()
    {
        std::cout << "------ LoopTimer Execution Summary ------" << std::endl;
        std::cout << "Elapsed Cycle: " << elapsedCycles() << std::endl;
        std::cout << "Elapsed Time: " << elapsedTime() << " in seconds."<< std::endl;
        std::cout << "Desired Loop Frequency: " << 1e9 / _ns_update_interval.count() << " hz" << std::endl;
        std::cout << "Actual Loop Frequency: " << elapsedCycles()/elapsedTime() << " hz" << std::endl;
        if (_avg_wait_time < 0)
            std::cout << "Overtime: " << _average_overtime_ms << " in  milliseconds" << std::endl;
        else
            std::cout << "Average Loop Waiting Time: " << _avg_wait_time << " in miliseconds"<< std::endl;
        std::cout << "Per Loop completion time: " << _loop_completion_time << " in milliseconds" << std::endl;
    }

    ~LoopTimer()
    {

    };
};

// #endif