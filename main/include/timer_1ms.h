#ifndef _TIMER_1MS
#define _TIMER_1MS

typedef struct
{
    uint32_t del;
    uint64_t run;
} stime_t;

void timer_1ms_init(void);
uint64_t get_time_ms(void);
void set_time_ms(uint64_t time_new);
void set_finish_time(uint32_t delay, stime_t *time);
bool is_time_out(stime_t *time);

#endif // !defined(_TIMER_1MS)
