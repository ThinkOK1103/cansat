#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
    double x;
    double y;
    double z;
} Point;

int main() {
    const char* input_filename = "data2.csv";
    const char* offset_filename = "offset.csv";
    const char* houi_filename = "houi.csv";

    FILE* file = fopen(input_filename, "r");
    if (!file) {
        perror("Error opening data2.csv");
        return 1;
    }

    size_t count = 0;
    size_t capacity = 1024;
    Point* data = (Point*)malloc(capacity * sizeof(Point));
    if (!data) {
        fprintf(stderr, "Memory allocation failed\n");
        fclose(file);
        return 1;
    }

    while (fscanf(file, "%lf,%lf,%lf", &data[count].x, &data[count].y, &data[count].z) == 3) {
        count++;
        if (count >= capacity) {
            capacity *= 2;
            Point* temp = (Point*)realloc(data, capacity * sizeof(Point));
            if (!temp) {
                fprintf(stderr, "Memory reallocation failed\n");
                free(data);
                fclose(file);
                return 1;
            }
            data = temp;
        }
    }
    fclose(file);

    if (count == 0) {
        fprintf(stderr, "No data read from file or file is empty.\n");
        free(data);
        return 0; 
    }

    double minX = data[0].x, maxX = data[0].x;
    double minY = data[0].y, maxY = data[0].y;
    double minZ = data[0].z, maxZ = data[0].z;

    for (size_t i = 1; i < count; i++) {
        if (data[i].x < minX) minX = data[i].x;
        if (data[i].x > maxX) maxX = data[i].x;
        if (data[i].y < minY) minY = data[i].y;
        if (data[i].y > maxY) maxY = data[i].y;
        if (data[i].z < minZ) minZ = data[i].z;
        if (data[i].z > maxZ) maxZ = data[i].z;
    }

    double offX = maxX + minX;
    double offY = maxY + minY;
    double offZ = maxZ + minZ;

    printf("%f\n", offX / 2.0);
    printf("%f\n", offY / 2.0);
    printf("%f\n", offZ / 2.0);

    Point* df_offset = (Point*)malloc(count * sizeof(Point));
    if (!df_offset) {
        fprintf(stderr, "Memory allocation failed for offset data\n");
        free(data);
        return 1;
    }

    for (size_t i = 0; i < count; i++) {
        df_offset[i].x = data[i].x - offX;
        df_offset[i].y = data[i].y - offY;
        df_offset[i].z = data[i].z - offZ;
    }

    FILE* offset_file = fopen(offset_filename, "w");
    if (!offset_file) {
        perror("Error opening offset.csv");
        free(data);
        free(df_offset);
        return 1;
    }

    for (size_t i = 0; i < count; i++) {
        fprintf(offset_file, "%.17g,%.17g,%.17g\n", df_offset[i].x, df_offset[i].y, df_offset[i].z);
    }
    fclose(offset_file);

    double* atan2_vals = (double*)malloc(count * sizeof(double));
    if (!atan2_vals) {
        fprintf(stderr, "Memory allocation failed for atan2 values\n");
        free(data);
        free(df_offset);
        return 1;
    }

    for (size_t i = 0; i < count; i++) {
        atan2_vals[i] = atan2(df_offset[i].y, df_offset[i].x);
    }

    double* houi = (double*)malloc(count * sizeof(double));
    if (!houi) {
        fprintf(stderr, "Memory allocation failed for houi values\n");
        free(data);
        free(df_offset);
        free(atan2_vals);
        return 1;
    }
    
    for (size_t i = 0; i < count; i++) {
        houi[i] = ((2.0 / M_PI) - atan2_vals[i]) * (180.0 / M_PI);
        if (houi[i] < 0) {
            houi[i] += 360.0;
        }
    }

    FILE* houi_file = fopen(houi_filename, "w");
    if (!houi_file) {
        perror("Error opening houi.csv");
        free(data);
        free(df_offset);
        free(atan2_vals);
        free(houi);
        return 1;
    }

    for (size_t i = 0; i < count; i++) {
        fprintf(houi_file, "%.17g\n", houi[i]);
    }
    fclose(houi_file);

    free(data);
    free(df_offset);
    free(atan2_vals);
    free(houi);

    return 0;
}