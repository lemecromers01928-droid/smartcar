/* Host-side replay runner for the exact embedded img_process.c implementation. */
#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "img_process.h"

static int next_token(FILE *file, char *buffer, size_t capacity)
{
    int ch;
    size_t used = 0;
    do
    {
        ch = fgetc(file);
        if (ch == '#')
        {
            while (ch != '\n' && ch != EOF) ch = fgetc(file);
        }
    } while (isspace(ch));
    if (ch == EOF) return 0;
    while (ch != EOF && !isspace(ch))
    {
        if (used + 1 < capacity) buffer[used++] = (char)ch;
        ch = fgetc(file);
    }
    buffer[used] = '\0';
    return used > 0;
}

static uint8_t *load_pgm(const char *path, int *rows, int *cols)
{
    FILE *file = fopen(path, "rb");
    char token[64];
    int ascii;
    int max_value;
    uint8_t *pixels;
    int count;
    int i;
    if (!file) return NULL;
    if (!next_token(file, token, sizeof(token))) goto fail;
    ascii = strcmp(token, "P2") == 0;
    if (!ascii && strcmp(token, "P5") != 0) goto fail;
    if (!next_token(file, token, sizeof(token))) goto fail;
    *cols = atoi(token);
    if (!next_token(file, token, sizeof(token))) goto fail;
    *rows = atoi(token);
    if (!next_token(file, token, sizeof(token))) goto fail;
    max_value = atoi(token);
    if (*rows <= 0 || *cols <= 0 || *rows > 255 || *cols > 255 || max_value != 255) goto fail;
    count = (*rows) * (*cols);
    pixels = (uint8_t *)malloc((size_t)count);
    if (!pixels) goto fail;
    if (ascii)
    {
        for (i = 0; i < count; ++i)
        {
            if (!next_token(file, token, sizeof(token)))
            {
                free(pixels);
                goto fail;
            }
            pixels[i] = (uint8_t)atoi(token);
        }
    }
    else if ((int)fread(pixels, 1, (size_t)count, file) != count)
    {
        free(pixels);
        goto fail;
    }
    fclose(file);
    return pixels;
fail:
    fclose(file);
    return NULL;
}

int main(int argc, char **argv)
{
    uint8_t *pixels;
    uint8_t *left;
    uint8_t *middle;
    uint8_t *right;
    short histogram[256];
    int rows = 0;
    int cols = 0;
    int threshold;
    int row_from_bottom;
    int sample_row;
    int valid;
    if (argc != 4)
    {
        fprintf(stderr, "usage: replay_runner image.pgm threshold_offset row_from_bottom\n");
        return 2;
    }
    pixels = load_pgm(argv[1], &rows, &cols);
    if (!pixels)
    {
        fprintf(stderr, "cannot read PGM image: %s\n", argv[1]);
        return 3;
    }
    left = (uint8_t *)calloc((size_t)rows, 1);
    middle = (uint8_t *)calloc((size_t)rows, 1);
    right = (uint8_t *)calloc((size_t)rows, 1);
    if (!left || !middle || !right) return 4;
    get_hist_gram(pixels, (unsigned short)rows, (unsigned short)cols, histogram);
    threshold = (int)get_threshold_otsu(histogram) + atoi(argv[2]);
    if (threshold < 1) threshold = 1;
    if (threshold > 254) threshold = 254;
    binaryzation_process(pixels, (unsigned short)rows, (unsigned short)cols, (unsigned int)threshold);
    auxiliary_process(pixels, (uint8_t)rows, (uint8_t)cols, (unsigned char)threshold, left, middle, right);
    row_from_bottom = atoi(argv[3]);
    sample_row = rows - row_from_bottom;
    if (sample_row < 0) sample_row = 0;
    if (sample_row >= rows) sample_row = rows - 1;
    valid = left[sample_row] != 0 || right[sample_row] != cols - 1;
    printf("threshold=%d,row=%d,left=%u,center=%u,right=%u,valid=%d\n",
           threshold, sample_row, left[sample_row], middle[sample_row], right[sample_row], valid);
    free(right);
    free(middle);
    free(left);
    free(pixels);
    return valid ? 0 : 5;
}
