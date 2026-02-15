# task1-v2
 usart+dma(echo server)
<img width="1276" height="357" alt="image" src="https://github.com/user-attachments/assets/ec2f0c9d-3ece-48d6-9753-54d8096c1b7c" />


был использован usart1 и dma2, а так же прерывание IDLE

так выглядит периодический опрос датчика(гироскоп) по I2C
<img width="899" height="562" alt="image" src="https://github.com/user-attachments/assets/69b4341a-b5a9-4021-929f-aa221209fffe" />

сырые значения гироскопа
<img width="2955" height="1272" alt="image" src="https://github.com/user-attachments/assets/724a13da-8dba-4263-bf9b-9ac3443f9d05" />

значения после добавления калибровки и альфа-фильтра(смещение ушло, размах колебаний уменьшился)
<img width="1320" height="571" alt="image" src="https://github.com/user-attachments/assets/5000ea9e-8c8f-46e7-bc93-332c2ce6abb9" />

значения гироскопа, после изменения работы i2c(добавления прерываний и dma)
<img width="2955" height="1272" alt="image" src="https://github.com/user-attachments/assets/343189ff-6e07-43c1-b45f-2b59380893b6" />

значения с акселерометра
<img width="2955" height="1272" alt="image" src="https://github.com/user-attachments/assets/c7b94c34-bd81-4b4f-8139-9ac9e34b9c80" />
