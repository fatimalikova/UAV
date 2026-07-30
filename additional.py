import atexit, math, time  # AZ: 3 hazır Python modulu gətirilir — atexit (proqram bitəndə funksiya işə salmaq), math (riyazi hesablamalar), time (vaxt/gözləmə üçün)
import dronekit as dk  # AZ: dronla əsas ünsiyyət kitabxanası, "dk" qısa adı ilə çağırılacaq
from dronekit import LocationGlobalRelative  # AZ: GPS koordinatı + nisbi hündürlüyü bir yerdə saxlayan "qutu" (obyekt tipi)
from pymavlink import mavutil  # AZ: DroneKit-in bacarmadığı əməliyyatlar üçün, birbaşa MAVLink protokolu ilə işləmək üçün

def wait_for(condition, timeout=10, interval=0.2, error=True):
    # AZ: Ümumi köməkçi funksiya — "bu şərt doğru olana qədər gözlə" məntiqini yerinə yetirir
    
    start = time.monotonic()  # AZ: hazırkı vaxtı qeyd edir, "başlanğıc nöqtəsi" kimi
    while not condition():  # AZ: "şərt" (condition) doğru olmadığı müddətcə dövr davam edir
        if timeout is not None and time.monotonic() - start >= timeout:  # AZ: keçən vaxt "timeout"u ötübsə
            if error:  # AZ: əgər xəta vermək tələb olunubsa (default: True)
                raise TimeoutError(f"Timed out after {timeout} seconds.")  # AZ: proqramı xəta ilə dayandırır
            return False  # AZ: xəta vermədən, sadəcə "uğursuz oldu" (False) qaytarır
        time.sleep(interval)  # AZ: növbəti yoxlamadan əvvəl qısa fasilə (default 0.2 saniyə)

def connect_drone(connect_str):
    # AZ: Drona (və ya simulyatora) qoşulan funksiya
   
    print(f"Connecting to {connect_str}.")  # AZ: ekrana "filan ünvana qoşulur" yazısı çıxarır
    vehicle = dk.connect(connect_str, wait_ready=True)  # AZ: əsl qoşulma — "vehicle" adlı obyekt yaranır, dronun "canlı təmsilçisi"
    atexit.register(vehicle.close)  # AZ: proqram bitəndə (hətta xəta ilə dayansa belə) avtomatik "vehicle.close()" çağırılsın deyə qeyd edir
    return vehicle  # AZ: yaradılan "vehicle" obyektini geri qaytarır ki, digər funksiyalar istifadə edə bilsin

def get_telemetry(vehicle):
    # AZ: Dronun hazırkı vəziyyəti haqqında məlumat toplayan funksiya
    
    bat = vehicle.battery  # AZ: batareya məlumatını (səviyyə, gərginlik) alır
    status = vehicle.system_status  # AZ: sistemin ümumi statusunu (məsələn "STANDBY", "ACTIVE") alır
    vel = vehicle.velocity  # AZ: sürəti [vx, vy, vz] siyahısı kimi alır (Şimal-Şərq-Aşağı istiqamətlərində, m/s)
    att = vehicle.attitude  # AZ: bucaqları (roll, pitch, yaw) radianla alır
    return {  # AZ: bütün toplanan məlumatı bir "lüğət" (dictionary) formasında qaytarır
            "mode": vehicle.mode.name if vehicle.mode else "UNKNOWN",  # AZ: hazırkı uçuş rejimi (məsələn "GUIDED"), yoxdursa "UNKNOWN"
            "armed": vehicle.armed,  # AZ: motorlar aktivdirmi (True/False)
            "lat": vehicle.location.global_relative_frame.lat,  # AZ: enlik (latitude) koordinatı
            "lon": vehicle.location.global_relative_frame.lon,  # AZ: uzunluq (longitude) koordinatı
            "alt": vehicle.location.global_relative_frame.alt,  # AZ: ev nöqtəsinə görə nisbi hündürlük (metr)
            "heading": vehicle.heading,  # AZ: kompas istiqaməti (dərəcə, 0-360)
            "battery_level": bat.level if bat else -1,  # AZ: batareya faizi, məlumat yoxdursa -1
            "gps": vehicle.gps_0.fix_type,  # AZ: GPS siqnalının keyfiyyət növü (rəqəm, nə qədər yüksəkdirsə bir o qədər yaxşı)
            "state": status.state if status else "UNKNOWN",  # AZ: sistem statusunun mətn forması
            "vx": vel[0] if vel else None,  # AZ: Şimala doğru sürət, "vel" yoxdursa None (bilinmir)
            "vy": vel[1] if vel else None,  # AZ: Şərqə doğru sürət
            "vz": vel[2] if vel else None,  # AZ: aşağıya doğru sürət (yuxarı qalxanda mənfi olur)
            "roll": att.roll if att else None,  # AZ: sağa-sola əyilmə bucağı
            "pitch": att.pitch if att else None,  # AZ: irəli-geri əyilmə bucağı
            "yaw": att.yaw if att else None,  # AZ: hansı istiqamətə "baxdığı" (kompas bucağının radian forması)
            }

def set_mode(vehicle, mode_name) -> None:
    # AZ: Uçuş rejimini dəyişən funksiya (məsələn "GUIDED", "LAND")
  
    print(f"Setting mode to {mode_name}...")  # AZ: ekrana "rejim filana dəyişdirilir" yazısı çıxarır
    vehicle.mode = dk.VehicleMode(mode_name)  # AZ: əsl dəyişiklik — rejimi tələb olunan adla qurur
    wait_for(lambda: vehicle.mode.name == mode_name)  # AZ: rejim həqiqətən dəyişənə qədər gözləyir


#   AZ: Aşağıdakı funksiyalar dronu HƏRƏKƏTƏ gətirən funksiyalardır.


def takeoff(vehicle, target_alt):
    # AZ: Dronu verilmiş hündürlüyə qaldıran funksiya
    
    print(f"Taking off to {target_alt}m...")  # AZ: "filan metrə qalxılır" yazısı
    vehicle.simple_takeoff(target_alt)  # AZ: DroneKit-in hazır qalxma funksiyası, dərhal qalxmağa başlayır
    wait_for(lambda: vehicle.location.global_relative_frame.alt >= target_alt * 0.95, timeout=20)  # AZ: hündürlüyün 95%-nə çatana qədər gözləyir (20 saniyə limitlə)
    print("Target altitude reached.")  # AZ: "hədəf hündürlüyə çatıldı" yazısı


def goto_position(vehicle, lat, lon, alt, groundspeed=None, timeout=60):
    # AZ: Dronu verilmiş GPS koordinatına uçuran YENİ funksiya
   
    target = LocationGlobalRelative(lat, lon, alt)  # AZ: "hara getmək istədiyimizi" bir obyekt kimi paketləyir
    print(f"Going to {lat}, {lon} at {alt}m...")  # AZ: "filan koordinata gedilir" yazısı
    if groundspeed:  # AZ: əgər sürət təyin olunubsa
        vehicle.simple_goto(target, groundspeed=groundspeed)  # AZ: həmin sürətlə hərəkətə başla
    else:  # AZ: sürət verilməyibsə
        vehicle.simple_goto(target)  # AZ: DroneKit-in defolt sürətiylə hərəkətə başla

    def distance_to_target():
        # AZ: daxili köməkçi funksiya — "hədəfə neçə metr qalıb?" sualına cavab verir
       
        current = vehicle.location.global_relative_frame  # AZ: dronun hazırkı GPS mövqeyini alır
        dlat = (target.lat - current.lat) * 111320  # AZ: enlik fərqini metrə çevirir (1 dərəcə ≈ 111320 metr)
        dlon = (target.lon - current.lon) * 111320 * math.cos(math.radians(current.lat))  # AZ: uzunluq fərqini metrə çevirir, enliyə görə düzəlişlə
        return math.sqrt(dlat**2 + dlon**2)  # AZ: Pifaqor teoremi ilə düz xətt məsafəsini hesablayır

    reached = wait_for(lambda: distance_to_target() < 1.0, timeout=timeout, error=False)  # AZ: məsafə 1 metrdən az olana qədər gözləyir, vaxt bitsə xəta vermir
    if reached:  # AZ: əgər hədəfə çatıbsa
        print("Reached target position.")  # AZ: "hədəf nöqtəyə çatıldı" yazısı
    else:  # AZ: çatmayıbsa (vaxt bitib)
        print("Warning: goto_position timed out before reaching target.")  # AZ: xəbərdarlıq yazısı


def release_payload(vehicle, servo_channel, release_pwm=1900, hold_pwm=1100, hold_time=1.0):
    # AZ: Yükü servo vasitəsilə buraxan YENİ funksiya
   
    print(f"Releasing payload on channel {servo_channel}...")  # AZ: "filan kanalda yük buraxılır" yazısı

    def set_servo(pwm):
        # AZ: daxili köməkçi funksiya — verilmiş PWM dəyərini verilmiş kanala göndərir
        msg = vehicle.message_factory.command_long_encode(  # AZ: xam MAVLink əmri "qurulur" (hələ göndərilmir)
                0, 0,  # AZ: hədəf sistem və komponent (0 = ignore/autopilot)
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # AZ: əmrin növü — "bu servonu bu dəyərə qur"
                0,  # AZ: təkrar təsdiq lazım deyil
                servo_channel,  # which servo output channel  # AZ: hansı servo kanalı
                pwm,            # PWM value to set, in microseconds  # AZ: göndəriləcək PWM dəyəri
                0, 0, 0, 0, 0)  # unused parameters  # AZ: istifadə olunmayan əlavə parametrlər
        vehicle.send_mavlink(msg)  # AZ: qurulan əmri həqiqətən drona göndərir

    set_servo(release_pwm)  # AZ: servonu "aç" (buraxma) mövqeyinə göndər
    time.sleep(hold_time)  # AZ: yükün düşməsi üçün qısa müddət gözlə
    set_servo(hold_pwm)  # AZ: servonu geri "bağlı" (istirahət) mövqeyinə qaytar
    print("Payload released.")  # AZ: "yük buraxıldı" yazısı


def spin_yaw(vehicle, angle, speed, report=False, timeout=15):
    # AZ: Dronu öz oxu ətrafında fırladan funksiya (əvvəldən var idi, indi timeout əlavə olunub)
   
    print(f"Spinning {angle} degrees at {speed} deg/s...")  # AZ: "filan dərəcə fırlanılır" yazısı
    msg = vehicle.message_factory.command_long_encode(  # AZ: xam MAVLink əmri qurulur
            0, 0, # These are target system and component. First 0 is ignored, second 0 indicates autopilot.  # AZ: hədəf sistem/komponent
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, # Command  # AZ: əmrin növü — "bucaq dəyiş"
            0, # 0 means no repeated confirmation is needed to execute this command  # AZ: təkrar təsdiq lazım deyil
            angle, # turning angle, in degrees  # AZ: neçə dərəcə dönəcək
            speed, # angular velocity, in degrees / second  # AZ: nə sürətlə dönəcək
            1, # turning direction, 1 - clockwise, -1 - counter-clockwise  # AZ: dönmə istiqaməti (saat əqrəbi istiqamətində)
            1, # 0 - absolute (exact compass angle), 1 - relative (turning degrees relative to previous angle)  # AZ: nisbi dönmə (əvvəlki bucağa görə)
            0, 0, 0) # Last three zeros are unused parameters  # AZ: istifadə olunmayan parametrlər
    vehicle.send_mavlink(msg) # We built and stored the message in msg variable, which this line sends to the drone  # AZ: əmri drona göndərir

#   AZ: Aşağıdakı sətirlər dronun HƏQİQƏTƏN lazımi qədər döndüyünü yoxlamaq üçündür.

    prev = vehicle.heading # vehicle.heading is the current compass angle  # AZ: əvvəlki (başlanğıc) kompas bucağı
    total = 0 # total it turned, begins with 0  # AZ: indiyə qədər dönülən ümumi bucaq, 0-dan başlayır
    start = time.monotonic()  # AZ: dövrün başlanğıc vaxtını qeyd edir (timeout üçün)

    while total < (angle - 5):  # AZ: tələb olunan bucağa (5 dərəcə tolerantla) çatana qədər dövr davam edir
        if time.monotonic() - start >= timeout:  # AZ: əgər timeout vaxtı ötübsə
            print("Warning: spin_yaw timed out, continuing anyway.")  # AZ: xəbərdarlıq yazır
            break  # AZ: dövrü məcburi dayandırır (sonsuz gözləmənin qarşısını alır)
        curr = vehicle.heading # read the current heading angle  # AZ: hazırkı kompas bucağını oxuyur
        if report: # Report parameter determines whether to print the current angle each time it checks. Disabled (false) by default.  # AZ: əgər "report" aktivdirsə
            print("Heading", curr)  # AZ: hazırkı bucağı ekrana çap edir

        """
        (İngiliscə izahat — 360 dərəcə keçiddə düzgün fərq hesablamaq üçün düstur)
        """
        diff = (curr - prev + 180) % 360 - 180  # AZ: iki bucaq arasındakı fərqi, 360 dərəcə "sıçrayışını" düzgün hesablayaraq tapır
        total += diff  # AZ: bu fərqi ümumi dönülən bucağa əlavə edir
        prev = curr  # AZ: "hazırkı"nı "əvvəlki" kimi saxlayır, növbəti dövr üçün
        time.sleep(0.1) # AZ: qısa fasilə, prosessoru həddindən artıq yükləməmək üçün

    print("Spin complete.")  # AZ: "fırlanma tamamlandı" yazısı


def land(vehicle):
    # AZ: Dronu təhlükəsiz endirən funksiya
   
    print("Landing...")  # AZ: "enilir" yazısı
    vehicle.mode = dk.VehicleMode("LAND")  # AZ: rejimi "LAND"-ə dəyişir, bu, avtomatik enməni başladır
    wait_for(lambda: not vehicle.armed, timeout=60)  # AZ: dron disarm olana (motor dayanana) qədər gözləyir (60 saniyə limitlə)
    print("Landed and disarmed.")  # AZ: "endi və disarm oldu" yazısı