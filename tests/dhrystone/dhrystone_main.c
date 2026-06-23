#include "dhry.h"

#define DATA_PORT   0x00e4

#define MARK_START  0xd0010000u
#define MARK_END    0xd0020000u

extern Rec_Pointer Ptr_Glob;
extern Rec_Pointer Next_Ptr_Glob;
extern int Int_Glob;
extern Boolean Bool_Glob;
extern char Ch_1_Glob;
extern char Ch_2_Glob;
extern int Arr_1_Glob[50];
extern int Arr_2_Glob[50][50];

static Rec_Type Record_Glob;
static Rec_Type Next_Record_Glob;

static void
io_out32 (unsigned short port, unsigned int value)
{
  __asm__ __volatile__ ("outl %0, %w1" :: "a"(value), "Nd"(port));
}

static unsigned int
fail_code (unsigned int code)
{
  return 0xd0000000u | code;
}

static void
initialize_dhrystone (void)
{
  int i;
  int j;

  for (i = 0; i < 50; i++)
  {
    Arr_1_Glob[i] = 0;
    for (j = 0; j < 50; j++)
      Arr_2_Glob[i][j] = 0;
  }

  Next_Ptr_Glob = &Next_Record_Glob;
  Ptr_Glob = &Record_Glob;

  Ptr_Glob->Ptr_Comp = Next_Ptr_Glob;
  Ptr_Glob->Discr = Ident_1;
  Ptr_Glob->variant.var_1.Enum_Comp = Ident_3;
  Ptr_Glob->variant.var_1.Int_Comp = 40;
  strcpy (Ptr_Glob->variant.var_1.Str_Comp,
          "DHRYSTONE PROGRAM, SOME STRING");

  Next_Ptr_Glob->Ptr_Comp = Ptr_Glob;
  Next_Ptr_Glob->Discr = Ident_1;
  Next_Ptr_Glob->variant.var_1.Enum_Comp = Ident_2;
  Next_Ptr_Glob->variant.var_1.Int_Comp = 40;
  strcpy (Next_Ptr_Glob->variant.var_1.Str_Comp,
          "DHRYSTONE PROGRAM, SOME STRING");

  Arr_2_Glob[8][7] = 10;
}

static unsigned int
validate_dhrystone (int number_of_runs,
                    int int_1_loc,
                    int int_2_loc,
                    int int_3_loc,
                    Enumeration enum_loc,
                    Str_30 str_1_loc,
                    Str_30 str_2_loc)
{
  if (Int_Glob != 5) return fail_code (0x01);
  if (Bool_Glob != true) return fail_code (0x02);
  if (Ch_1_Glob != 'A') return fail_code (0x03);
  if (Ch_2_Glob != 'B') return fail_code (0x04);
  if (Arr_1_Glob[8] != 7) return fail_code (0x05);
  if (Arr_2_Glob[8][7] != number_of_runs + 10) return fail_code (0x06);

  if (Ptr_Glob->Discr != Ident_1) return fail_code (0x10);
  if (Ptr_Glob->variant.var_1.Enum_Comp != Ident_3) return fail_code (0x11);
  if (Ptr_Glob->variant.var_1.Int_Comp != 17) return fail_code (0x12);
  if (strcmp (Ptr_Glob->variant.var_1.Str_Comp,
              "DHRYSTONE PROGRAM, SOME STRING") != 0) return fail_code (0x13);

  if (Next_Ptr_Glob->Discr != Ident_1) return fail_code (0x20);
  if (Next_Ptr_Glob->variant.var_1.Enum_Comp != Ident_2) return fail_code (0x21);
  if (Next_Ptr_Glob->variant.var_1.Int_Comp != 18) return fail_code (0x22);
  if (strcmp (Next_Ptr_Glob->variant.var_1.Str_Comp,
              "DHRYSTONE PROGRAM, SOME STRING") != 0) return fail_code (0x23);

  if (int_1_loc != 5) return fail_code (0x30);
  if (int_2_loc != 13) return fail_code (0x31);
  if (int_3_loc != 7) return fail_code (0x32);
  if (enum_loc != Ident_2) return fail_code (0x33);
  if (strcmp (str_1_loc, "DHRYSTONE PROGRAM, 1'ST STRING") != 0) return fail_code (0x34);
  if (strcmp (str_2_loc, "DHRYSTONE PROGRAM, 2'ND STRING") != 0) return fail_code (0x35);

  return 0;
}

int
dhrystone_main (void)
{
  One_Fifty Int_1_Loc;
  register One_Fifty Int_2_Loc;
  One_Fifty Int_3_Loc;
  register char Ch_Index;
  Enumeration Enum_Loc;
  Str_30 Str_1_Loc;
  Str_30 Str_2_Loc;
  register int Run_Index;
  register int Number_Of_Runs;

  Number_Of_Runs = DHRY_ITERS;
  initialize_dhrystone ();
  strcpy (Str_1_Loc, "DHRYSTONE PROGRAM, 1'ST STRING");

  io_out32 (DATA_PORT, MARK_START | (unsigned int)(Number_Of_Runs & 0xffff));

  for (Run_Index = 1; Run_Index <= Number_Of_Runs; ++Run_Index)
  {
    Proc_5 ();
    Proc_4 ();

    Int_1_Loc = 2;
    Int_2_Loc = 3;
    strcpy (Str_2_Loc, "DHRYSTONE PROGRAM, 2'ND STRING");
    Enum_Loc = Ident_2;
    Bool_Glob = ! Func_2 (Str_1_Loc, Str_2_Loc);

    while (Int_1_Loc < Int_2_Loc)
    {
      Int_3_Loc = 5 * Int_1_Loc - Int_2_Loc;
      Proc_7 (Int_1_Loc, Int_2_Loc, &Int_3_Loc);
      Int_1_Loc += 1;
    }

    Proc_8 (Arr_1_Glob, Arr_2_Glob, Int_1_Loc, Int_3_Loc);
    Proc_1 (Ptr_Glob);

    for (Ch_Index = 'A'; Ch_Index <= Ch_2_Glob; ++Ch_Index)
    {
      if (Enum_Loc == Func_1 (Ch_Index, 'C'))
      {
        Proc_6 (Ident_1, &Enum_Loc);
        strcpy (Str_2_Loc, "DHRYSTONE PROGRAM, 3'RD STRING");
        Int_2_Loc = Run_Index;
        Int_Glob = Run_Index;
      }
    }

    Int_2_Loc = Int_2_Loc * Int_1_Loc;
    Int_1_Loc = Int_2_Loc / Int_3_Loc;
    Int_2_Loc = 7 * (Int_2_Loc - Int_3_Loc) - Int_1_Loc;
    Proc_2 (&Int_1_Loc);
  }

  io_out32 (DATA_PORT, MARK_END | (unsigned int)(Number_Of_Runs & 0xffff));

  return validate_dhrystone (Number_Of_Runs,
                             Int_1_Loc,
                             Int_2_Loc,
                             Int_3_Loc,
                             Enum_Loc,
                             Str_1_Loc,
                             Str_2_Loc);
}
