#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <time.h>


#define SWIM_DEV            "/dev/swim"
#define SWIM_IOCTL_RESET    1

#define MAX_SIZE            10240


static unsigned char buf[MAX_SIZE] = {0};
static unsigned char filename[256] = {0};


void print_usage(void)
{
    printf("usage:\n");
    printf("\treset:\n");
    printf("\t\ts\n");
    printf("\twrite:\n");
    printf("\t\tw [addr] [data] [size]\n");
    printf("\tread:\n");
    printf("\t\tr [addr] [size]\n");
    printf("\tdownload:\n");
    printf("\t\td [addr] [filename]\n");
    printf("\tquit:\n");
    printf("\t\tq\n");
}


void print_data(unsigned char *buf, unsigned int size)
{
    int i;

    for (i=0; i<size; i++)
    {
        if ((i % 8 == 0) && i)
        {
            printf(" ");
        }
        if ((i % 16 == 0) && i)
        {
            printf("\n");
        }
        printf("%02X ", *buf++);
    }
    printf("\n");
}


main()
{
    int fd = 0;
    char cmd = 0;
    unsigned int addr, size;
    unsigned char data;

    fd = open(SWIM_DEV, O_RDWR);
    if (fd < 0)
    {
        printf(SWIM_DEV" open error!\n");
        return -1;
    }

    print_usage();

    while (cmd != 'q')
    {
        printf("> ");
        gets(buf);

        cmd = buf[0];
        if (cmd == 's')
        {
            printf("reset? (y/n): ");
            gets(buf);
            if (buf[0] == 'y')
            {
                printf("reset\n");
            }
        }
        else if (cmd == 'w')
        {
            sscanf(buf, "%c %x %x %x\n", &cmd, &addr, (unsigned int *)&data, &size);
            printf("write addr=0x%x data=0x%x size=0x%x? (y/n): ", addr, data, size);
            gets(buf);
            if (buf[0] == 'y')
            {
                if (lseek(fd, addr, SEEK_SET) < 0)
                {
                    printf("seek error!\n");
                    continue;
                }
                if (write(fd, buf, size) != size)
                {
                    printf("write error!\n");
                    continue;
                }
                printf("write done.\n");
            }
        }
        else if (cmd == 'r')
        {
            sscanf(buf, "%c %x %x\n", &cmd, &addr, &size);
            printf("read addr=0x%x size=0x%x? (y/n): ", addr, size);
            gets(buf);
            if (buf[0] == 'y')
            {
                if (lseek(fd, addr, SEEK_SET) < 0)
                {
                    printf("seek error!\n");
                    continue;
                }
                if (read(fd, buf, size) != size)
                {
                    printf("read error!\n");
                    continue;
                }
                print_data(buf, size);
            }
        }
        else if (cmd == 'd')
        {
            sscanf(buf, "%c %x %s\n", &cmd, &addr, filename);
            printf("download addr=0x%x file=%s? (y/n): ", addr, filename);
            gets(buf);
            if (buf[0] == 'y')
            {
                int bin_fd = 0;
                struct stat stat_buf;

                stat(filename, &stat_buf);
                size = stat_buf.st_size;
                printf("%s file size: %d\n", filename, (unsigned int)stat_buf.st_size);
                printf("%s last modify time: %s\n", filename, ctime(&stat_buf.st_mtime));

                bin_fd = open(SWIM_DEV, O_RDONLY);
                if (read(bin_fd, buf, size) != size)
                {
                    printf("read error!\n");
                    close(bin_fd);
                    continue;
                }

                if (lseek(fd, addr, SEEK_SET) < 0)
                {
                    printf("seek error!\n");
                    close(bin_fd);
                    continue;
                }
                if (write(fd, buf, size) != size)
                {
                    printf("write error!\n");
                    close(bin_fd);
                    continue;
                }
                close(bin_fd);
                printf("download OK\n");
            }
        }
    }

    close(fd);
}
