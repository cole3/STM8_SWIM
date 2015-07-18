#include "stm8_swim.h"

/********************************************************
 * demo:
 *     host               target
 * -------------       ------------
 * |       rst |>----->| rst      |
 * |   swim_in |<----+-| swim     |
 * |  swim_out |>---/  |          |
 * -------------       ------------
 *******************************************************/


#define SWIM_CMD_LEN                    3
#define SWIM_CMD_SRST                   0x00
#define SWIM_CMD_ROTF                   0x01
#define SWIM_CMD_WOTF                   0x02
#define SWIM_MAX_RESEND_CNT             20


unsigned int return_line;
static swim_input input, *p_input = &input;
static swim_method method, *p_method = &method;

static void swim_ndelay(unsigned int ns)
{
    p_input->ndelay(ns);
}

static void swim_udelay(unsigned int us)
{
    while (us--)
    {
        swim_ndelay(1000);
    }
}

static void swim_mdelay(unsigned int ms)
{
    while (ms--)
    {
        swim_udelay(1000);
    }
}

static void swim_send_bit(unsigned char bit)
{
    // low speed
    if (bit)
    {
        p_input->io_set(SWIM, LOW);
        swim_ndelay(250); // 2*(1/8M) = 250ns
        p_input->io_set(SWIM, HIGH);
        swim_ndelay(2500); // 20*(1/8M) = 2500ns
    }
    else
    {
        p_input->io_set(SWIM, LOW);
        swim_ndelay(2500); // 2*(1/8M) = 2500ns
        p_input->io_set(SWIM, HIGH);
        swim_ndelay(250); // 20*(1/8M) = 250ns
    }
}

static char swim_rvc_bit(void)
{
    unsigned int i;
    unsigned char p = 0, m = 1;
    
    // low speed
#define ACK_CHK_TIMEOUT 1000
    for (i=0; i<ACK_CHK_TIMEOUT; i++)
    {
        swim_ndelay(125);
        if (p_input->io_get(SWIM) == LOW)
        {
            m = 0;
            p++;
        }

        if (m == 0 && p_input->io_get(SWIM) == HIGH)
        {   
            return (p <= 8) ? 1 : 0;
        }
    }
    return -1;
}

static void swim_send_ack(unsigned char ack)
{
    swim_ndelay(2750);
    swim_send_bit(ack);
}

static char swim_rvc_ack(void)
{
    return swim_rvc_bit();
}


static swim_ret swim_send_unit(unsigned char data, unsigned char len, unsigned int retry_cnt)
{
    char i;
    unsigned char p, m;
    char ack = 0;

    swim_ndelay(2750);

    do {
        swim_send_bit(0);
        p = 0;
        for (i=len-1; i>=0; i--)
        {
            m =(data >> i) & 1;
            swim_send_bit(m);
            p += m;
        }
        // parity bit
        swim_send_bit((p & 1));
        ack = swim_rvc_ack();
        if (ack == -1)
        {
            return SWIM_TIMEOUT;
        }
    } while (!ack && retry_cnt--);

    return ack ? SWIM_OK : SWIM_FAIL;
}

static swim_ret swim_rcv_unit(unsigned char *data, unsigned char len)
{
    unsigned int i;
    unsigned char s = 0, m = 0, p = 0, cp = 0;
    char ack = 0;

    for (i=0; i<len+2; i++)
    {
        ack = swim_rvc_bit();
        if (ack == -1)
        {
            return SWIM_TIMEOUT;
        }
        if (i == 0)
        {
            s = ack;
        }
        else if (i == len + 1)
        {
            p = ack;
        }
        else
        {
            m <<= 1;
            m |= ack;
            cp += ack;
        }
    }

    if (s == 1 && p == (cp & 1))
    {
        ack = 1;
    }

    swim_send_ack(ack);

    return ack ? SWIM_OK : SWIM_FAIL;
}


static swim_ret swim_reset(void)
{
    return swim_send_unit(SWIM_CMD_SRST, SWIM_CMD_LEN, SWIM_MAX_RESEND_CNT);
}

static swim_ret swim_read(unsigned int addr, unsigned char *buf, unsigned int size)
{
    unsigned char cur_len, i;
    unsigned int cur_addr = addr;
    swim_ret ret = SWIM_OK;

    if (!buf)
    {
        return SWIM_FAIL;
    }

    while (size)
    {
        cur_len = (size > 255) ? 255 : size;

        ret = swim_send_unit(SWIM_CMD_ROTF, SWIM_CMD_LEN, SWIM_MAX_RESEND_CNT);
        if (ret)
        {
            return_line = __LINE__;
            break;
        }
        ret = swim_send_unit(cur_len, 8, 0);
        if (ret)
        {
            return_line = __LINE__;
            break;
        }
        ret = swim_send_unit((cur_addr >> 16) & 0xFF, 8, 0);
        if (ret)
        {
            return_line = __LINE__;
            break;
        }
        ret = swim_send_unit((cur_addr >> 8) & 0xFF, 8, 0);
        if (ret)
        {
            return_line = __LINE__;
            break;
        }
        ret = swim_send_unit((cur_addr >> 0) & 0xFF, 8, 0);
        if (ret)
        {
            return_line = __LINE__;
            break;
        }
        for (i = 0; i < cur_len; i++)
        {
            ret = swim_rcv_unit(buf++, 8);
            if (ret)
            {
                return_line = __LINE__;
                break;
            }
        }
        if (ret)
        {
            break;
        }

        cur_addr += cur_len;
        size -= cur_len;
    }

    return ret;
}

static swim_ret swim_write(unsigned int addr, unsigned char *buf, unsigned int size)
{
    unsigned char cur_len, i;
    unsigned int cur_addr = addr;
    swim_ret ret = SWIM_OK;

    if (!buf)
    {
        return SWIM_FAIL;
    }

    while (size)
    {
        cur_len = (size > 255) ? 255 : size;

        ret = swim_send_unit(SWIM_CMD_WOTF, SWIM_CMD_LEN, SWIM_MAX_RESEND_CNT);
        if (ret)
        {
            return_line = __LINE__;
            break;
        }
        ret = swim_send_unit(cur_len, 8, 0);
        if (ret)
        {
            return_line = __LINE__;
            break;
        }
        ret = swim_send_unit((cur_addr >> 16) & 0xFF, 8, 0);
        if (ret)
        {
            return_line = __LINE__;
            break;
        }
        ret = swim_send_unit((cur_addr >> 8) & 0xFF, 8, 0);
        if (ret)
        {
            return_line = __LINE__;
            break;
        }
        ret = swim_send_unit((cur_addr >> 0) & 0xFF, 8, 0);
        if (ret)
        {
            return_line = __LINE__;
            break;
        }
        for (i = 0; i < cur_len; i++)
        {
            ret = swim_send_unit(*buf++, 8, SWIM_MAX_RESEND_CNT);
            if (ret)
            {
                return_line = __LINE__;
                break;
            }
        }
        if (ret)
        {
            break;
        }

        cur_addr += cur_len;
        size -= cur_len;
    }

    return ret;
}

static swim_ret swim_entry(void)
{
    unsigned char i, m;
    swim_ret ret = SWIM_FAIL;
    unsigned char buf[1];

    p_input->io_set(SWIM, HIGH);
    p_input->io_set(RST, HIGH);
    swim_mdelay(50);
    p_input->io_set(RST, LOW);
    swim_mdelay(10);
    p_input->io_set(SWIM, LOW);
    swim_udelay(100);

    for (i=0; i<4; i++)
    {
        p_input->io_set(SWIM, HIGH);
        swim_udelay(500);
        p_input->io_set(SWIM, LOW);
        swim_udelay(500);
    }
    for (i=0; i<4; i++)
    {
        p_input->io_set(SWIM, HIGH);
        swim_udelay(250);
        p_input->io_set(SWIM, LOW);
        swim_udelay(250);
    }

    p_input->io_set(SWIM, HIGH);
    m = 1;

#define RST_CHK_TIMEOUT 1000
    // about 16 us
    for (i=0; i<RST_CHK_TIMEOUT; i++)
    {
        swim_ndelay(500);
        if (p_input->io_get(SWIM) == LOW)
        {
            m = 0;
        }

        if (m == 0 && p_input->io_get(SWIM) == HIGH)
        {
            ret = SWIM_OK;
        }
    }

    if (ret)
    {
        return_line = __LINE__;
        return ret;
    }

    swim_mdelay(10);
    ret = swim_reset();
    if (ret)
    {
        return_line = __LINE__;
        return ret;
    }

    swim_mdelay(30);
    buf[0]=0xA0;
    ret = swim_write(0x00007F80, buf, 1); 
    if (ret)
    {
        return_line = __LINE__;
        return ret;
    }

    swim_mdelay(10);
    p_input->io_set(RST, HIGH);
    swim_mdelay(10);
    

    return ret;
}


swim_method *swim_register(swim_input *swim_input)
{
    if (!swim_input)
    {
        return 0;
    }

    input = *swim_input;

    p_method->entry = swim_entry;
    p_method->reset = swim_reset;
    p_method->read = swim_read;
    p_method->write = swim_write;

    return p_method;
}

void swim_unregister(swim_method *method)
{

}
