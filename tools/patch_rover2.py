p = r'C:\Users\dell\Desktop\rover\rover esp\rover\rover.ino'
s = open(p, encoding='utf-8').read()
a = 'void readTelemetry() {'
assert a in s
s = s.replace(a, '''void handleTelem() {
  char b[420];
  snprintf(b, sizeof b,
    "{\\"q\\":[%.4f,%.4f,%.4f,%.4f],\\"g\\":[%.4f,%.4f,%.4f],\\"a\\":[%.4f,%.4f,%.4f],\\"us\\":%u,\\"tof\\":[%u,%u],"
    "\\"w\\":[%d,%d,%d,%d],\\"pos\\":[%.1f,%.1f],\\"hd\\":%.1f,\\"r\\":%.1f,\\"p\\":%.1f,\\"cal\\":%u,\\"mc\\":%.2f,\\"t\\":%lu}",
    fusion.q0, fusion.q1, fusion.q2, fusion.q3,
    fusion.gyroRaw[0] - fusion.gyroBias[0], fusion.gyroRaw[1] - fusion.gyroBias[1], fusion.gyroRaw[2] - fusion.gyroBias[2],
    lastAcc[0], lastAcc[1], lastAcc[2], ultraMm, tofMm[0], tofMm[1],
    wheelMmS[0], wheelMmS[1], wheelMmS[2], wheelMmS[3], odomX, odomY, heading, roll, pitch,
    (fusion.gyroCal ? 1 : 0) | (fusion.magCal ? 2 : 0) | (fusion.levelCal ? 4 : 0) | (fusion.stationary ? 8 : 0),
    fusion.magCoverage, millis());
  server.send(200, "application/json", b);
}

void readTelemetry() {''', 1)
b = '  server.on("/status", handleStatus);'
assert b in s
s = s.replace(b, b + '\n  server.on("/telem", handleTelem);\n  server.on("/calib", []() {\n    String c = server.arg("c");\n    if (c == "level") fusion.calibrateLevel(prefs);\n    else if (c == "magreset") fusion.resetMagCal();\n    else if (c == "save") fusion.save(prefs);\n    else if (c == "zero") fusion.zeroYaw();\n    ok();\n  });', 1)
open(p, 'w', encoding='utf-8').write(s)
print('ok')
