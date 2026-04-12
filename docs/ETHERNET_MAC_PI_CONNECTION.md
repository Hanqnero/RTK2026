# Подключение Mac ↔ Raspberry Pi по Ethernet (прямое соединение)

## Схема сети

```
Mac (en7 / USB-C Ethernet)  <──ethernet──>  Raspberry Pi 5 (eth0)
        192.168.2.1                               192.168.2.2
         bridge100
```

Mac раздаёт интернет Pi через **Internet Sharing** (бридж `bridge100`).

---

## 1. Настройка на Mac

### 1.1 Включить Internet Sharing

1. **System Settings → General → Sharing**
2. Найти **Internet Sharing**, нажать кнопку **ℹ️**
3. **Share your connection from:** Wi-Fi (или основной интерфейс с интернетом)
4. **To computers using:** Thunderbolt Ethernet (или USB LAN адаптер — тот, что подключён к Pi)
5. Включить переключатель **Internet Sharing**

После включения Mac создаёт интерфейс `bridge100` с IP **192.168.2.1** и запускает DHCP-сервер.
Pi автоматически получает IP **192.168.2.2** (или следующий в диапазоне 192.168.2.x).

### 1.2 Проверить на Mac

```bash
ifconfig bridge100
# должно быть: inet 192.168.2.1 netmask 0xffffff00

ping 192.168.2.2       # Pi должна отвечать
ssh pi@192.168.2.2     # SSH работает из обычного Terminal
```

---

## 2. Настройка на Raspberry Pi

### 2.1 Проверить IP

```bash
# на Pi:
ip addr show eth0
# должно быть: inet 192.168.2.2/24
```

### 2.2 Добавить default route (если Pi не видит интернет)

DHCP от Mac не всегда прописывает default gateway. Проверить:

```bash
ip route show
# если нет строки "default via 192.168.2.1" — добавить вручную:
sudo /usr/sbin/ip route add default via 192.168.2.1
```

> Используй полный путь `/usr/sbin/ip` — sudo на Ubuntu не включает `/usr/sbin` в PATH.

Проверить интернет на Pi:

```bash
ping -c3 8.8.8.8
```

### 2.3 Если ping 8.8.8.8 не работает — прокси через Mac

Mac по умолчанию делает NAT для bridge100, но если не работает, поднять Squid на Mac:

```bash
# На Mac — запустить squid-прокси в Docker:
docker run -d --name squid -p 3128:3128 ubuntu/squid

# На Pi — установить прокси:
export http_proxy=http://192.168.2.1:3128
export https_proxy=http://192.168.2.1:3128

# Проверить:
curl -x http://192.168.2.1:3128 https://example.com
```

---

## 3. SSH и туннели

### Подключение к Pi

```bash
ssh pi@192.168.2.2
```

### SSH-туннель для Foxglove WebSocket

Docker-контейнеры на Mac **не могут** напрямую достучаться до Pi по DDS/UDP (Docker VM изолирует bridge100). Для Foxglove используй `foxglove_bridge` на Pi и SSH-туннель:

```bash
# Перенаправить порт 8765 Pi → localhost:8765 на Mac:
ssh -f -N -L 8765:localhost:8765 pi@192.168.2.2
```

После этого Foxglove Studio на Mac подключается к `ws://localhost:8765`.

---

## 4. Запуск контейнера на Pi

```bash
ssh pi@192.168.2.2

# Перейти в репозиторий:
cd ~/RTK2026

# Запустить контейнер (если не запущен):
docker run -d --name rtk2026 \
  --privileged \
  -v /dev:/dev \
  --network host \
  rtk2026:latest
```

Проверка Foxglove внутри контейнера:

```bash
docker logs rtk2026 2>&1 | grep foxglove
# ожидается строка вида:
# foxglove_bridge: WebSocket server started on port 8765
```

Если нужен интерактивный shell без остановки основного процесса:

```bash
docker exec -it rtk2026 bash
```

---

## 5. Типичные проблемы

| Проблема | Причина | Решение |
|----------|---------|---------|
| VS Code Terminal: `No route to host` | VS Code изолирует сеть | Использовать Terminal.app |
| Pi не в сети | DHCP ещё не присвоил IP | Подождать 10–15 сек, проверить `ip addr` |
| `sudo ip`: command not found | sudo's secure_path не включает `/usr/sbin` | Использовать `sudo /usr/sbin/ip` |
| Pi нет интернета | Нет default route | `sudo /usr/sbin/ip route add default via 192.168.2.1` |
| Foxglove не подключается к `ws://localhost:8765` | На Pi не запущен `foxglove_bridge` или нет SSH-туннеля | Проверить `docker logs rtk2026 \| grep foxglove` и `ssh -L 8765:localhost:8765 pi@192.168.2.2` |
| Foxglove WebSocket ошибка SSL | HTTPS страница → ws:// блокируется браузером | Использовать Foxglove Desktop |
