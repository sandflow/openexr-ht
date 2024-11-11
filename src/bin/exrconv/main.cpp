#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ImfArray.h"
#include "ImfCompression.h"
#include "ImfHeader.h"
#include "ImfRgbaFile.h"
#include "ImfFrameBuffer.h"
#include <ImfNamespace.h>
#include <OpenEXRConfig.h>

namespace IMF = OPENEXR_IMF_NAMESPACE;
using namespace OPENEXR_IMF_NAMESPACE;
using namespace IMATH_NAMESPACE;

#define MAX_PATH_LENGTH 512

typedef struct _filename_parts
{
  char prefix[MAX_PATH_LENGTH];
  char suffix[MAX_PATH_LENGTH];
  unsigned int number_of_digits;
} filename_parts_t;

typedef struct _commandline_args
{
  char input_filename[MAX_PATH_LENGTH];
  char output_filename[MAX_PATH_LENGTH];
  filename_parts_t input_filename_parts;
  filename_parts_t output_filename_parts;
  unsigned int start_frame;
  unsigned int end_frame;
  char compression_string[MAX_PATH_LENGTH];
  bool is_process_framerange;
} commandline_args_t;

static const char *compression_string_table[NUM_COMPRESSION_METHODS] = {
"NO_COMPRESSION", //  = 0 // no compression
"RLE_COMPRESSION",	// = 1 // run length encoding
"ZIPS_COMPRESSION",// = 2	// zlib compression, one scan line at a time
"ZIP_COMPRESSION", // = 3,	// zlib compression, in blocks of 16 scan lines
"PIZ_COMPRESSION", // = 4,	// piz-based wavelet compression
"PXR24_COMPRESSION", // = 5,	// lossy 24-bit float compression
"B44_COMPRESSION", // = 6,	// lossy 4-by-4 pixel block compression,
                            // fixed compression rate
"B44A_COMPRESSION", // = 7,	// lossy 4-by-4 pixel block compression,
                            // flat fields are compressed more
"DWAA_COMPRESSION", // = 8,       // lossy DCT based compression, in blocks
                              // of 32 scanlines. More efficient for partial
                              // buffer access.
"DWAB_COMPRESSION", //  = 9,       // lossy DCT based compression, in blocks
                              // of 256 scanlines. More efficient space
                              // wise and faster to decode full frames
                              // than DWAA_COMPRESSION.
"HT_COMPRESSION",
"HT256_COMPRESSION",
"HTK_COMPRESSION",
"HTK256_COMPRESSION",
};

void print_argument_list(int argc, char* argv[])
{
  fprintf(stderr, "argument list:\n");
  for (int i = 0; i < argc; i++)
    fprintf(stderr, "\t argv[%d] = %s\n", i, argv[i]);
  return;
}

void print_allowed_compression_strings()
{
  for (int i = 0; i < (int)NUM_COMPRESSION_METHODS; i++)
  {
    fprintf(stderr, "\t %s\n", compression_string_table[i]);
  }

  return;
}

void print_usage(int argc, char* argv[])
{
  fprintf(stderr, "This program converts exr images to exr images with different image compression\n");
  fprintf(stderr, "USAGE: %s \n", argv[0]);
  fprintf(stderr, "\nREQUIRED ARGUMENTS:\n");
  fprintf(stderr, " -i <input_filename>  - exr filename - use # symbols for input sequence frame number digits, e.g.  input.######.exr\n");
  fprintf(stderr, " -o <output_filename> - exr filename - use # symbols for input sequence frame number digits, e.g. output.######.exr\n");
  fprintf(stderr, "\nOPTIONAL ARGUMENTS:\n");
  fprintf(stderr, " -c <compression_string> - use this to specify different image compression for the output\n");
  fprintf(stderr, "\tsupported compression strings:\n");
  print_allowed_compression_strings();
  fprintf(stderr, " -s <start_frame>\n");
  fprintf(stderr, " -e <end_frame>\n");

  fprintf(stderr, "\nUSAGE EXAMPLE: %s -i input.exr -o output.exr -c PIZ_COMPRESSION\n", argv[0]);
  fprintf(stderr, "\nUSAGE EXAMPLE: %s -i input.####.exr -o output.######.exr -s 3 -e 450 -c ZIP_COMPRESSION\n", argv[0]);


  if (argc != 1)
  {
    fprintf(stderr, "\n");
    print_argument_list(argc, argv);
  }

  fprintf(stderr, "\nVERSION INFO: This software executable was compiled on %s at %s\n", __DATE__, __TIME__);

  return;
}

int process_commandline_args(int argc, char* argv[], commandline_args_t *args)
{
  // set defaults
  memset(args, 0, sizeof(commandline_args_t));

  bool is_i_parsed = false;
  bool is_o_parsed = false;
  bool is_c_parsed = false;
  bool is_s_parsed = false;
  bool is_e_parsed = false;

  if (argc == 1)
  {
    print_usage(argc, argv);
    return -1;
  }

  for (int i = 1; i < argc; i++)
  {
    int args_remaining = argc - i;

    // usage
    if (strcmp(argv[i], "-u") == 0)
    {
      print_usage(argc, argv);
      exit(1);
    }

    // input filename
    else if (strcmp(argv[i], "-i") == 0)
    {
      if (args_remaining <= 1)
      {
        fprintf(stderr, "USAGE ERROR in function %s of file %s on line %d:\n -i missing filename argument\n", __FUNCTION__, __FILE__, __LINE__);
        return -1;
      }
      else if (true == is_i_parsed)
      {
        fprintf(stderr, "USAGE ERROR in function %s of file %s on line %d:\n -i argument has already been processed, -i should only be used once\n", __FUNCTION__, __FILE__, __LINE__);
        return -1;
      }
      else
      {
        strcpy(args->input_filename, argv[i + 1]);
        i = i + 1;
        is_i_parsed = true;
      }
    }

    // output filename
    else if (strcmp(argv[i], "-o") == 0)
    {
      if (args_remaining <= 1)
      {
        fprintf(stderr, "USAGE ERROR in function %s of file %s on line %d:\n -o missing filename argument\n", __FUNCTION__, __FILE__, __LINE__);
        return -1;
      }
      else if (true == is_o_parsed)
      {
        fprintf(stderr, "USAGE ERROR in function %s of file %s on line %d:\n -o argument has already been processed, -o should only be used once\n", __FUNCTION__, __FILE__, __LINE__);
        return -1;
      }
      else
      {
        strcpy(args->output_filename, argv[i + 1]);
        i = i + 1;
        is_o_parsed = true;
      }
    }

    // start frame 
    else if (strcmp(argv[i], "-s") == 0)
    {
      if (args_remaining <= 1)
      {
        fprintf(stderr, "USAGE ERROR in function %s of file %s on line %d:\n -s missing start frame argument\n", __FUNCTION__, __FILE__, __LINE__);
        return -1;
      }
      else if (true == is_s_parsed)
      {
        fprintf(stderr, "USAGE ERROR in function %s of file %s on line %d:\n -s argument has already been processed, -s should only be used once\n", __FUNCTION__, __FILE__, __LINE__);
        return -1;
      }
      else
      {
        args->start_frame = (unsigned int)atoi(argv[i + 1]);
        i = i + 1;
        is_s_parsed = true;
      }
    }

    // end frame 
    else if (strcmp(argv[i], "-e") == 0)
    {
      if (args_remaining <= 1)
      {
        fprintf(stderr, "USAGE ERROR in function %s of file %s on line %d:\n -e missing end frame argument\n", __FUNCTION__, __FILE__, __LINE__);
        return -1;
      }
      else if (true == is_e_parsed)
      {
        fprintf(stderr, "USAGE ERROR in function %s of file %s on line %d:\n -e argument has already been processed, -e should only be used once\n", __FUNCTION__, __FILE__, __LINE__);
        return -1;
      }
      else
      {
        args->end_frame = (unsigned int)atoi(argv[i + 1]);
        i = i + 1;
        is_e_parsed = true;
      }
    }

    // compression
    else if (strcmp(argv[i], "-c") == 0)
    {
      if (args_remaining <= 1)
      {
        fprintf(stderr, "USAGE ERROR in function %s of file %s on line %d:\n -c missing compression string argument\n", __FUNCTION__, __FILE__, __LINE__);
        return -1;
      }
      else if (true == is_c_parsed)
      {
        fprintf(stderr, "USAGE ERROR in function %s of file %s on line %d:\n -c argument has already been processed, -c should only be used once\n", __FUNCTION__, __FILE__, __LINE__);
        return -1;
      }
      else
      {
        strcpy(args->compression_string, argv[i + 1]);
        i = i + 1;
        is_c_parsed = true;
      }
    }

    // unrecognized arguments
    else
    {
      fprintf(stderr, "COMMANDLINE PROCESSING ERROR: argv[%d] = %s, this is an unrecognized command-line argument\n", i, argv[i]);
      exit(1);
    }
  } // end of argv[] processing loop


    // check that input filename and output filename are processed
  if (true != is_i_parsed)
  {
    fprintf(stderr, "USAGE ERROR in function %s of file %s on line %d:\n -o is missing but is a required argument\n", __FUNCTION__, __FILE__, __LINE__);
    return -1;
  }
  if (true != is_o_parsed)
  {
    fprintf(stderr, "USAGE ERROR in function %s of file %s on line %d:\n -o is missing but is a required argument\n", __FUNCTION__, __FILE__, __LINE__);
    return -1;
  }

  // if one of end-frame/start-frame arguments is processed, then make sure all of them are processed
  if ((true == is_s_parsed) || (true == is_e_parsed))
  {
    args->is_process_framerange = true;

    if ((true != is_s_parsed) || (true != is_e_parsed))
    {
      fprintf(stderr, "USAGE ERROR in function %s of file %s on line %d:\n", __FUNCTION__, __FILE__, __LINE__);
      fprintf(stderr, " if one of the following arguments are used, \n -s -e, then all the arguments must be used.\n");

      if (true == is_s_parsed)
        fprintf(stderr, "-sf1 was used\n");
      else
        fprintf(stderr, "-sf1 was not used\n");

      if (true == is_e_parsed)
        fprintf(stderr, "-ef1 was used\n");
      else
        fprintf(stderr, "-ef1 was not used\n");

      return -1;
    }
  }

  return 0;
}

void split_filename_with_symbols_into_parts( char* input_string_with_symbols_to_find, char* prefix, char* suffix, unsigned int* number_of_digits)
{
  // this function splits an input string with sequential symbols into prefix, suffix and number of digits
  // for example test.####.tif will be split into prefix = test. suffix = .tif and number_of_digits = 4

  const char* symbol_to_find = "#";

  *number_of_digits = 0;
  bool keep_searching = true;
  char* found_search_result = NULL;
  while (true == keep_searching && *number_of_digits < 64 )
  {
    char symbols_to_find[MAX_PATH_LENGTH] = { '\0' };
    for (size_t i = 1; i <= (size_t)*number_of_digits + 1; i++)
    {
      strncat(symbols_to_find, symbol_to_find, 1);
    }
    char* search_result = strstr(input_string_with_symbols_to_find, symbols_to_find);
    if (search_result)
    {
      found_search_result = search_result;
      *number_of_digits = *number_of_digits + 1;
    }
    else
    {
      keep_searching = false;
    }

  };

  if (*number_of_digits > 64)
  {
    fprintf(stderr, "WARNING on line %d of file %s in function %s:\n number_of_digits = %d is greater than 64\n",
      __LINE__, __FILE__, __FUNCTION__, *number_of_digits);
  }

  if (NULL != found_search_result)
  {
    // find prefix
    size_t number_of_prefix_chars = found_search_result - input_string_with_symbols_to_find;
    strncpy(prefix, input_string_with_symbols_to_find, number_of_prefix_chars);
    // find suffix
    size_t input_string_length = strlen(input_string_with_symbols_to_find);
    strncpy(suffix, &found_search_result[*number_of_digits], (size_t)(input_string_length - *number_of_digits));
  }
  else
  {
    // since no symbols were found, just copy the filename into prefix
    size_t number_of_prefix_chars = strnlen(input_string_with_symbols_to_find, MAX_PATH_LENGTH);
    strncpy(prefix, input_string_with_symbols_to_find, number_of_prefix_chars);
  }
  
  return;
}

void make_filename(char* output_string, const char* prefix, const char* suffix, const unsigned int number_of_digits, const unsigned int frame_number)
{
  size_t max_string_copy = sizeof(prefix) + sizeof(suffix) + (size_t)number_of_digits;

  if (number_of_digits > 0)
  {
    char string_formatting[MAX_PATH_LENGTH] = { '\0' };
    // make a string with the appropriate number of digits, like "%s%06d%s"
    snprintf(string_formatting, sizeof(string_formatting), "%%s%%0%dd%%s", number_of_digits);

    snprintf(output_string, max_string_copy, string_formatting, prefix, frame_number, suffix);
  }
  else
  { 
    snprintf(output_string, max_string_copy, "%s%s", prefix, suffix);
  }

  return;
}

int check_commandline_args(commandline_args_t *args)
{

  if (true == args->is_process_framerange)
  {
    if (args->end_frame < args->start_frame)
    {
      fprintf(stderr, "USAGE ERROR in function %s of file %s on line %d:\n", __FUNCTION__, __FILE__, __LINE__);
      fprintf(stderr, " args->end_frame should not be less than args->start_frame\n");
      fprintf(stderr, " args->start_frame = %d args->end_frame = %d\n", args->start_frame, args->end_frame);
      return -1;
    }
  }

  // split filenames into parts
  split_filename_with_symbols_into_parts( args->input_filename, 
    args->input_filename_parts.prefix, 
    args->input_filename_parts.suffix, 
    &(args->input_filename_parts.number_of_digits));
  split_filename_with_symbols_into_parts( args->output_filename, 
    args->output_filename_parts.prefix, 
    args->output_filename_parts.suffix, 
    &(args->output_filename_parts.number_of_digits));

  // check that the we have access to the range of input files specified
  const unsigned int number_of_frames_to_process = args->end_frame - args->start_frame + 1;

  // check the specified input frames
  for (unsigned int frame_index = 0; frame_index < number_of_frames_to_process; frame_index++)
  {
    if (0 == frame_index)
    {
      fprintf(stderr, "Start checking %d input files . . .\n", number_of_frames_to_process);
    }

    // make image filename
    char input_filename[2*MAX_PATH_LENGTH] = { '\0' };
    //sprintf(input_filename, args->input_filename, frame_index + args->start_frame);
    make_filename( input_filename, 
      args->input_filename_parts.prefix, 
      args->input_filename_parts.suffix, 
      args->input_filename_parts.number_of_digits, 
      frame_index + args->start_frame);

    // try to open image file
    {
      FILE *input_file = NULL;
      input_file = fopen(input_filename, "rb");
      if (NULL == input_file)
      {
        fprintf(stderr, "ERROR in file on line %d of %s in function %s(): unable to open input filename = %s for binary reading\n",
          __LINE__, __FILE__, __FUNCTION__, input_filename);
        return false;
      }
      else
      {
        // we can open the image file OK
        fclose(input_file);
      }
    }

  }
  fprintf(stderr, "Finished checking %d input files\n", number_of_frames_to_process);

  if ( '\0' == args->compression_string[0])
  {
    strcpy(args->compression_string, compression_string_table[0]);
    fprintf(stderr, "USAGE WARNING in function %s of file %s on line %d:\n", __FUNCTION__, __FILE__, __LINE__);
    fprintf(stderr, " -c argument not provided, using c = %s compression value.\n", args->compression_string);
  }

  return 0;
}

Compression get_compression_from_compression_string(const char* compression_string)
{
  for (int i = 0; i < (int)NUM_COMPRESSION_METHODS; i++)
  {
    if (0 == strcmp(compression_string, compression_string_table[i]))
    {
      return (Compression) i;
    }
  }

  fprintf(stderr, "USAGE ERROR in function %s of file %s on line %d:\n", __FUNCTION__, __FILE__, __LINE__);
    fprintf(stderr, " -c %s is not on the list of supported compression strings. \nThe supported list of compression values is shown below:\n", compression_string);
    print_allowed_compression_strings();
    fprintf(stderr, "Exiting.\n");
    exit(-1);

  return NO_COMPRESSION;
}


int main(int argc, char* argv[])
{
  // process command line arguments
  commandline_args_t args;
  int process_command_args_ok = process_commandline_args(argc, argv, &args);
  if (process_command_args_ok != 0)
  {
    fprintf(stderr, "\n");
    print_argument_list(argc, argv);
    fprintf(stderr, "\n");
    fprintf(stderr, "use -u argument to print usage info\n");
    exit(-1);
  }

  // check command arguments
  int check_command_args_ok = check_commandline_args(&args);
  if (check_command_args_ok != 0)
  {
    fprintf(stderr, "USAGE ERROR: problem with checking command-line args\n");
    fprintf(stderr, "\n");
    print_argument_list(argc, argv);
    fprintf(stderr, "use -u argument to print usage info\n");
    exit(1);
  }

  Compression selected_compression = get_compression_from_compression_string(args.compression_string);

  unsigned int number_of_frames_to_process = args.end_frame - args.start_frame + 1;
  for (unsigned int frame_index = 0; frame_index < number_of_frames_to_process; frame_index++)
  {
    // make image filenames
    char input_filename[2*MAX_PATH_LENGTH] = { '\0' };
    char output_filename[2*MAX_PATH_LENGTH] = { '\0' };
    //sprintf(input_filename, args.input_filename, frame_index + args.start_frame);
    //sprintf(output_filename, args.output_filename, frame_index + args.start_frame);
    make_filename( input_filename, 
      args.input_filename_parts.prefix, 
      args.input_filename_parts.suffix, 
      args.input_filename_parts.number_of_digits, 
      frame_index + args.start_frame);
    make_filename( output_filename, 
      args.output_filename_parts.prefix, 
      args.output_filename_parts.suffix, 
      args.output_filename_parts.number_of_digits, 
      frame_index + args.start_frame);

    RgbaInputFile i_file(input_filename);

    Box2i         dw = i_file.dataWindow ();
    int           width = dw.max.x - dw.min.x + 1;
    int           height = dw.max.y - dw.min.y + 1;

    Array2D<Rgba> pixels (height, width);

    i_file.setFrameBuffer (&pixels[-dw.min.x][-dw.min.y], 1, width);
    i_file.readPixels (dw.min.y, dw.max.y);

    Header        header = i_file.header();
    header.compression () = selected_compression;


    RgbaOutputFile o_file(output_filename, header, WRITE_RGB);
    o_file.setFrameBuffer(&pixels[-dw.min.x][-dw.min.y], 1, width);
    o_file.writePixels(height);

  }

  return 0;
}