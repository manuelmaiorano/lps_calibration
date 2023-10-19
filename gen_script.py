for i in range(8):
    for j in range(8):
        if i != j:

            print(f"LOG_ADD(LOG_UINT16, dist{i}-{j}, &logReciprocalDistances[{i}][{j}])")