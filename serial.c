#include <string.h>
#include <stdint.h>
#include "CRC16.h"

//Немного абстрации
#define UART_MODE

#ifdef UART_MODE
#include "uart_port.h"
#else						//UART_MODE
#include "can_port.h"
#endif						//UART_MODE


#define MAX_FRAME_SIZE 255						//Ограничим максимальную длину посылки

uint8_t txDataBuf[MAX_FRAME_SIZE];

//==========================================================
//  Управляющие символы потока (байт-стаффинг).
//  FEND - начало посылки 
//  FESC, TFEND, TFESC - резервные байты для подмены (байт-стаффинга)
//==========================================================
enum {
    FEND  = 0xB4,
    FESC  = 0xB5,
    TFEND = 0xB6,
    TFESC = 0xB7
};		

//==========================================================
//  Стадии приема
//==========================================================
typedef char TWait; enum {
           WAIT_FEND,     // ожидание приема FEND
           WAIT_NBT0,     // ожидание приема количества байт в пакете
           WAIT_NBT1,     // ожидание приема количества байт в пакете
           WAIT_DATA,     // прием данных
           WAIT_CRC0,     // ожидание окончания приема CRC
           WAIT_CRC1     // ожидание окончания приема CRC
    } ;

//==========================================================
// Структура для принимаемой посылки
//==========================================================
#pragma pack(1)
typedef struct {
    uint8_t        		   		    sta;        // состояние процесса приема пакета
    uint8_t                          pre;        // предыдущий принятый байт
    uint8_t                          addr;       // адрес, с которым сравнивается принятый
    uint16_t                         nbt;        // сколько байт в пакете
    uint8_t                          *ptr;       // куда класть принимаемые данные
    uint16_t                         crc;        // контрольная сумма принимаемого пакета
    uint16_t                         ccrc;
    uint16_t                         ctr;        // счетчик байтов данных
    uint32_t                         time;       // время приема
} TdRx;           
#pragma pack()	

TdRx pCtl [MAX_FRAME_SIZE];


//==========================================================
//  Сформировать и передать пакет в порт (абстрактный CAN или UART)
//==========================================================
int WriteData (void *ptr, uint8_t addr, uint16_t size) {
    uint8_t              *dptr = &txDataBuf;
    uint8_t              *sptr = (uint8_t*)ptr;
    uint16_t             framesize = 0;
    uint16_t             crc = 0;
    uint8_t              data;

    if (size > MAX_FRAME_SIZE) return -1;                 // Слишком длинный кусок.
    // Начало пакета всегда FEND.                               
    data = FEND;                                                
    crc = CRC16 (data, crc);                                    
    *dptr++ = data;                                             
    framesize++;                                                
    // Добавить адрес.                                          
    data = addr | 0x80;                                         // Длинный пакет.
    // Добавить размер.
    AddWide (&size, 2, dptr, framesize, crc);
    // Добавить данные.
    AddWide (sptr, size, dptr, framesize, crc);
    // Добавить CRC. После этого весь пакет собран.
    AddWide (&crc, sizeof(crc), dptr, framesize, 0);
    // Передать все сформированое в COM-port/CAN-порт.
    return WritePort(dptr, framesize);							//объявлен в хедере uart_port.h или can_port.h
}

//==========================================================
// Добавление байтов данных к посылке с учетом байт стаффинга
//==========================================================
void AddWide (void *ptr, uint16_t size, uint8_t* &dptr, uint16_t &framesize, uint16_t &crc) {
    uint8_t              data;
    uint8_t              *sptr = (uint8_t*)ptr;

    while (size--) {
        data = *sptr++;
        crc = CRC16 (data, crc);
        if (data == FEND) {
            *dptr++ = FESC;
            *dptr++ = TFEND;
            framesize += 2;
        } else if (data == FESC) {
            *dptr++ = FESC;
            *dptr++ = TFESC;
            framesize += 2;
        } else {
            *dptr++ = data;
            framesize++;
        }
    }
}


//==========================================================
// Обработчик входного потока. Выделяет и сохраняет
// пакеты.
// Возвращает -1, если байт данных обработан, в противном
// случае, возвращает сам байт данных. 
//==========================================================
short OnRxHandler (uint8_t data_byte) {
    TdRx     	             *pRx = &pCtl->Rx;
    uint8_t                 Pre;

    //------------------
    if (data_byte == FEND) {                    //если обнаружено начало фрейма,
        pRx->pre = data_byte;                   //то сохранение пре-байта,
        pRx->crc = 0;			   	            //инициализация CRC,
        pRx->crc = CRC16 (data_byte, pRx->crc);
        pRx->sta = WAIT_NBT0;                   //сброс указателя данных,
        return -1;                              //выход (байт обработан)
    }

    if (pRx->sta == WAIT_FEND) {                //-----> если ожидание FEND,
        return data_byte;                       //то выход (байт не обработан)
    }

    Pre = pRx->pre;                             //сохранение старого пре-байта
    pRx->pre = data_byte;                       //обновление пре-байта
    if (Pre == FESC) {                          //если пре-байт равен FESC,
        if (data_byte == TFESC)                 //а байт данных равен TFESC,
            data_byte = FESC;                   //то заменить его на FESC
        else if (data_byte == TFEND)            //если байт данных равен TFEND,
            data_byte = FEND;                   //то заменить его на FEND
        else {
            pRx->sta = WAIT_FEND;               //для всех других значений байта данных,
            return -1;                          //выход (байт обработан)
        }
    } else {
        if (data_byte == FESC)                  //если байт данных равен FESC, он просто
            return -1;                          //запоминается в пре-байте. выход (байт обработан)
    }
    //------------------
    switch (pRx->sta) {
        
        case WAIT_NBT0:
            pRx->crc = CRC16 ( data_byte, pRx->crc );
            pRx->nbt = data_byte;
            pRx->sta = WAIT_NBT1;
            break;

        case WAIT_NBT1:                         
            pRx->crc = CRC16 ( data_byte, pRx->crc );
            pRx->ctr = 0;
            pRx->nbt |= (U16)(data_byte << 8);
            if (pRx->nbt) {
                pRx->sta = WAIT_DATA;
            } else pRx->sta = WAIT_CRC0;
            break;

        case WAIT_DATA:
            pRx->crc = CRC16 ( data_byte, pRx->crc );
            *(pRx->ptr + pRx->ctr) = data_byte;
            if (++pRx->ctr >= pRx->nbt) pRx->sta = WAIT_CRC0; // Блок данных принят.
            break;

        case WAIT_CRC0:
            pRx->ccrc = data_byte;
            pRx->sta = WAIT_CRC1;
            break;

        case WAIT_CRC1:                         
            // Запоминиаем входящий адрес.
            pRx->ccrc |= (U16)(data_byte<<8);
            if (pRx->crc == pRx->ccrc) {
                pCtl->cmd = pRx->cmd;           // Новый пакет принят верно.
            } else {
                //SET ERROR FLAG
            }
            pRx->sta = WAIT_FEND;
            break;
    }
    return -1;
}