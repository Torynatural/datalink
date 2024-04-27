#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "protocol.h"
#include "datalink.h"

#define MAX_SEQ 63
#define PKT_LEN 256
#define NR_BUFS ((MAX_SEQ + 1) / 2)
#define DATA_MAX_TIME 5000
#define ACK_MAX_TIME 285

#define inc(k) k = (k + 1) & MAX_SEQ

struct FRAME {
    unsigned char kind; /* FRAME_DATA */
    unsigned char ack;
    unsigned char seq;
    unsigned char data[PKT_LEN];
    unsigned char padding[4];
};

struct Packet {
    unsigned char data[PKT_LEN];
    size_t len;
} ;

static unsigned char frame_nr = 0, nbuffered;
static unsigned char frame_expected = 0;
static bool phl_ready = 0;
static struct Packet out_buf[NR_BUFS]; // 接收窗口
static struct Packet in_buf[NR_BUFS];  // 发送窗口
static unsigned int arrived[NR_BUFS];   // 标记帧是否到达的bit map


static bool between(unsigned int a, unsigned int b, unsigned int c){
    return a <= b && b < c || c < a && a <= b || b < c && c < a;
}

static void put_frame(unsigned char* frame, int len)
{
    *(unsigned int*)(frame + len) = crc32(frame, len);
    send_frame(frame, len + 4);
    phl_ready = 0;
}

static void send_data_frame(unsigned char fk, unsigned int frame_nr, unsigned int frame_expected)
{
    struct FRAME s;
    s.kind = fk;
    s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1);

    if(fk == FRAME_DATA)
    {
        s.seq = frame_nr;
        memcpy(s.data, out_buf[frame_nr % NR_BUFS].data, out_buf[frame_nr % NR_BUFS].len);
        dbg_frame("Send DATA %d %d, ID %d\n", s.seq, s.ack, *(short*)s.data);
        put_frame((unsigned char*)&s, 3 + out_buf[frame_nr % NR_BUFS].len);
        start_timer(frame_nr % NR_BUFS, DATA_MAX_TIME);
    }
    else 
    {
        if (fk == FRAME_NAK)
        {
            s.ack = frame_expected;
            dbg_frame("Send NAK %d\n", frame_expected);
        }
        else
            dbg_frame("Send ACK %d\n", s.ack);
        send_frame((unsigned char*)&s, 3);
    }
    stop_ack_timer();
}

int main(int argc, char** argv)
{
    int event, arg;
    struct FRAME f;
    int len ;
    int i;
    unsigned int nbuffered = 0;
    unsigned int ack_expected = 0;       // 期望收到ACK的帧序号，发送窗口的下界
    unsigned int next_frame_to_send = 0; // 下一个发出的帧序号，发送窗口的上界+1
    unsigned int frame_expected = 0;     // 期望收到的帧序号
    unsigned int too_far = NR_BUFS;      // 接收窗口的上界+1

    protocol_init(argc, argv);
    lprintf("Designed by nyt, build: " __DATE__"  "__TIME__"\n");

    disable_network_layer();

    for (;;) {
        event = wait_for_event(&arg);

        switch (event) {
        case NETWORK_LAYER_READY:
            nbuffered++;
            out_buf[next_frame_to_send % NR_BUFS].len = get_packet(out_buf[next_frame_to_send % NR_BUFS].data);
            send_data_frame(FRAME_DATA, next_frame_to_send, frame_expected);
            inc(next_frame_to_send);
            break;

        case PHYSICAL_LAYER_READY:
            phl_ready = 1;
            break;

        case FRAME_RECEIVED:
            len = recv_frame((unsigned char*)&f, sizeof f);
            if (len < 5 && len != 3)
                break;
            if (len >= 5 || crc32((unsigned char*)&f, len) != 0) {
                dbg_event("**** Receiver Error, Bad CRC Checksum\n");
                send_data_frame(FRAME_NAK, 0, f.seq);
                break;
            }

            if (f.kind == FRAME_ACK)
                dbg_frame("Recv ACK  %d\n", f.ack);

            if (f.kind == FRAME_DATA) {
                dbg_frame("Recv DATA %d %d, ID %d\n", f.seq, f.ack, *(short*)f.data);
                start_ack_timer(ACK_MAX_TIME);
                if (between(frame_expected, f.seq, too_far) && !arrived[f.seq % NR_BUFS])
                {
                    arrived[f.seq % NR_BUFS] = true;
                    memcpy(in_buf[f.seq % NR_BUFS].data, f.data, len - 7);
                    in_buf[f.seq % NR_BUFS].len = len - 7;
                    while (arrived[frame_expected % NR_BUFS]) // 将接收缓冲区中连续的已收到的帧发送到网络层
                    {
                        put_packet(in_buf[frame_expected % NR_BUFS].data, in_buf[frame_expected % NR_BUFS].len);
                        arrived[frame_expected % NR_BUFS] = false;
                        inc(frame_expected);
                        inc(too_far);
                        start_ack_timer(ACK_MAX_TIME);
                    }
                }
            }

            if (f.kind == FRAME_NAK && between(ack_expected, f.ack, next_frame_to_send)) // 发送特定的NAK帧
            {
                dbg_frame("Recv NAK %d\n", f.ack);
                send_data_frame(FRAME_DATA, f.ack, frame_expected);
                break;
            }

            while (between(ack_expected, f.ack, next_frame_to_send)) {
                nbuffered--;
                stop_timer(ack_expected % NR_BUFS);
                inc(ack_expected);
            }
            break;

        case DATA_TIMEOUT:
            dbg_event("---- DATA %d timeout\n", arg);
            if (!between(ack_expected, arg, next_frame_to_send)) // 计时器中的序号只有一半，要将序号转化到窗口区间内
                arg += NR_BUFS;
            send_data_frame(FRAME_ACK, arg, frame_expected);
            break;
        

        case ACK_TIMEOUT:
            dbg_event("---- ACK %d timeout ----\n", frame_expected);
            send_data_frame(FRAME_ACK, 0, frame_expected);
            break;
    }
        
        if (nbuffered < NR_BUFS && phl_ready)
            enable_network_layer();
        else
            disable_network_layer();
    }
    
}
