package kr.dronegcs.app;

import java.io.ByteArrayOutputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketTimeoutException;
import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.Map;

/**
 * 같은 Wi-Fi의 "이름.local"(예: dmdpi.local)을 IP로 바꿈.
 * 안드로이드는 .local 이름을 못 찾는 기기가 많아서, 파이(avahi)에 직접 mDNS로 물어봄 (RFC 6762 단발 질의:
 * 5353이 아닌 포트에서 보내면 파이가 그 포트로 바로 답함 → 멀티캐스트 수신 권한이 없어도 됨).
 */
final class LocalNet {
    private LocalNet() { }

    private static final Map<String, Object[]> cache = new HashMap<>();   // 이름 → {IP, 찾은 시각}
    private static final long KEEP_MS = 5 * 60_000;

    /** 질의 패킷: 이름의 A 레코드, QU(바로 답해 달라) 표시 */
    static byte[] query(String name, int id) {
        ByteArrayOutputStream b = new ByteArrayOutputStream();
        b.write(id >> 8); b.write(id); b.write(0); b.write(0);   // id, flags
        b.write(0); b.write(1); for (int i = 0; i < 6; i++) b.write(0);   // 질문 1개
        for (String label : name.split("\\.")) {
            byte[] l = label.getBytes(StandardCharsets.UTF_8);
            b.write(l.length); b.write(l, 0, l.length);
        }
        b.write(0);
        b.write(0); b.write(1);          // A
        b.write(0x80); b.write(1);       // QU + IN
        return b.toByteArray();
    }

    /** 이름 건너뛰기 (압축 포인터 포함) → 다음 위치 */
    private static int skipName(byte[] p, int i) {
        while (i < p.length) {
            int n = p[i] & 0xff;
            if (n == 0) return i + 1;
            if ((n & 0xc0) == 0xc0) return i + 2;
            i += n + 1;
        }
        return p.length;
    }

    /** 응답에서 첫 번째 IPv4 주소, 없으면 null */
    static InetAddress parseA(byte[] p, int len) {
        try {
            if (len < 12) return null;
            int qd = ((p[4] & 0xff) << 8) | (p[5] & 0xff);
            int an = ((p[6] & 0xff) << 8) | (p[7] & 0xff);
            int ns = ((p[8] & 0xff) << 8) | (p[9] & 0xff);
            int ar = ((p[10] & 0xff) << 8) | (p[11] & 0xff);
            int i = 12;
            for (int k = 0; k < qd; k++) i = skipName(p, i) + 4;
            for (int k = 0; k < an + ns + ar && i < len; k++) {
                i = skipName(p, i);
                if (i + 10 > len) return null;
                int type = ((p[i] & 0xff) << 8) | (p[i + 1] & 0xff);
                int rdlen = ((p[i + 8] & 0xff) << 8) | (p[i + 9] & 0xff);
                i += 10;
                if (type == 1 && rdlen == 4 && i + 4 <= len)
                    return InetAddress.getByAddress(new byte[]{p[i], p[i + 1], p[i + 2], p[i + 3]});
                i += rdlen;
            }
        } catch (Exception ignored) { }
        return null;
    }

    /** mDNS로 물어봄. sock은 호출한 쪽이 Wi-Fi에 묶어서 넘겨 줌 (모바일 데이터로 나가지 않게) */
    static InetAddress mdns(String name, DatagramSocket sock, InetAddress group, int port) throws Exception {
        int id = (int) (System.nanoTime() & 0x7fff) | 1;
        byte[] q = query(name, id), buf = new byte[1500];
        sock.setSoTimeout(700);
        for (int tries = 0; tries < 3; tries++) {
            sock.send(new DatagramPacket(q, q.length, group, port));
            long end = System.currentTimeMillis() + 700;
            while (System.currentTimeMillis() < end) {
                DatagramPacket r = new DatagramPacket(buf, buf.length);
                try { sock.receive(r); } catch (SocketTimeoutException e) { break; }
                InetAddress a = parseA(buf, r.getLength());
                if (a != null) return a;
            }
        }
        return null;
    }

    static synchronized InetAddress cached(String name) {
        Object[] c = cache.get(name);
        return c != null && System.currentTimeMillis() - (long) c[1] < KEEP_MS ? (InetAddress) c[0] : null;
    }

    static synchronized void remember(String name, InetAddress a) {
        if (a == null) cache.remove(name); else cache.put(name, new Object[]{a, System.currentTimeMillis()});
    }

    /** 같은 Wi-Fi(사설망) 주소인지: 이 통로로는 집 안·현장 기기에만 연결 (인터넷 주소로는 안 감) */
    static boolean isLocal(InetAddress a) {
        return a.isSiteLocalAddress() || a.isLinkLocalAddress() || a.isLoopbackAddress()
                || (a.getAddress().length == 4 && (a.getAddress()[0] & 0xff) == 100 && (a.getAddress()[1] & 0xc0) == 64);   // 100.64/10 (일부 핫스팟)
    }
}
