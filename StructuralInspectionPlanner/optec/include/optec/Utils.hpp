/*
 *	This file is part of qpOASES.
 *
 *	qpOASES -- An Implementation of the Online Active Set Strategy.
 *	Copyright (C) 2007-2014 by Hans Joachim Ferreau, Andreas Potschka,
 *	Christian Kirches et al. All rights reserved.
 *
 *	qpOASES is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 2.1 of the License, or (at your option) any later version.
 *
 *	qpOASES is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *	See the GNU Lesser General Public License for more details.
 *
 *	You should have received a copy of the GNU Lesser General Public
 *	License along with qpOASES; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
 *	\file include/qpOASES/Utils.hpp
 *	\author Hans Joachim Ferreau, Andreas Potschka, Christian Kirches
 *	\version 3.0beta
 *	\date 2007-2014
 *
 *	Declaration of some utility functions for working with qpOASES.
 */


#ifndef QPOASES_UTILS_HPP
#define QPOASES_UTILS_HPP


#include <optec/MessageHandling.hpp>


BEGIN_NAMESPACE_QPOASES


/** Prints a (possibly named) vector.
 * \return SUCCESSFUL_RETURN */
returnValue print(	const real_t* const v,	/**< Vector to be printed. */
					int n,					/**< Length of vector. */
					const char* name = 0	/**< Name of vector. */
					);

/** Prints a (possibly named) permuted vector.
 * \return SUCCESSFUL_RETURN */
returnValue print(	const real_t* const v,		/**< Vector to be printed. */
					int n,						/**< Length of vector. */
					const int* const V_idx,		/**< Pemutation vector. */
					const char* name = 0		/**< Name of vector. */
					);

/** Prints a (possibly named) matrix.
 * \return SUCCESSFUL_RETURN */
returnValue print(	const real_t* const M,	/**< Matrix to be printed. */
					int nrow,				/**< Row number of matrix. */
					int ncol,				/**< Column number of matrix. */
					const char* name = 0	/**< Name of matrix. */
					);

/** Prints a (possibly named) permuted matrix.
 * \return SUCCESSFUL_RETURN */
returnValue print(	const real_t* const M,		/**< Matrix to be printed. */
					int nrow,					/**< Row number of matrix. */
					int ncol	,				/**< Column number of matrix. */
					const int* const ROW_idx,	/**< Row pemutation vector. */
					const int* const COL_idx,	/**< Column pemutation vector. */
					const char* name = 0		/**< Name of matrix. */
					);

/** Prints a (possibly named) index array.
 * \return SUCCESSFUL_RETURN */
returnValue print(	const int* const index,	/**< Index array to be printed. */
					int n,					/**< Length of index array. */
					const char* name = 0	/**< Name of index array. */
					);


/** Prints a string to desired output target (useful also for MATLAB output!).
 * \return SUCCESSFUL_RETURN */
returnValue myPrintf(	const char* s	/**< String to be written. */
						);


/** Prints qpOASES copyright notice.
 * \return SUCCESSFUL_RETURN */
returnValue printCopyrightNotice( );


/** Reads a real_t matrix from file.
 * \return SUCCESSFUL_RETURN \n
 		   RET_UNABLE_TO_OPEN_FILE \n
		   RET_UNABLE_TO_READ_FILE */
returnValue readFromFile(	real_t* data,				/**< Matrix to be read from file. */
							int nrow,					/**< Row number of matrix. */
							int ncol,					/**< Column number of matrix. */
							const char* datafilename	/**< Data file name. */
							);

/** Reads a real_t vector from file.
 * \return SUCCESSFUL_RETURN \n
 		   RET_UNABLE_TO_OPEN_FILE \n
		   RET_UNABLE_TO_READ_FILE */
returnValue readFromFile(	real_t* data,				/**< Vector to be read from file. */
							int n,						/**< Length of vector. */
							const char* datafilename	/**< Data file name. */
							);

/** Reads an integer (column) vector from file.
 * \return SUCCESSFUL_RETURN \n
 		   RET_UNABLE_TO_OPEN_FILE \n
		   RET_UNABLE_TO_READ_FILE */
returnValue readFromFile(	int* data,					/**< Vector to be read from file. */
							int n,						/**< Length of vector. */
							const char* datafilename	/**< Data file name. */
							);


/** Writes a real_t matrix into a file.
 * \return SUCCESSFUL_RETURN \n
 		   RET_UNABLE_TO_OPEN_FILE  */
returnValue writeIntoFile(	const real_t* const data,		/**< Matrix to be written into file. */
							int nrow,						/**< Row number of matrix. */
							int ncol,						/**< Column number of matrix. */
							const char* datafilename,		/**< Data file name. */
							BooleanType append = BT_FALSE	/**< Indicates if data shall be appended if the file already exists (otherwise it is overwritten). */
							);

/** Writes a real_t vector into a file.
 * \return SUCCESSFUL_RETURN \n
 		   RET_UNABLE_TO_OPEN_FILE  */
returnValue writeIntoFile(	const real_t* const data,		/**< Vector to be written into file. */
							int n,							/**< Length of vector. */
							const char* datafilename,		/**< Data file name. */
							BooleanType append = BT_FALSE	/**< Indicates if data shall be appended if the file already exists (otherwise it is overwritten). */
							);

/** Writes an integer (column) vector into a file.
 * \return SUCCESSFUL_RETURN \n
 		   RET_UNABLE_TO_OPEN_FILE */
returnValue writeIntoFile(	const int* const integer,		/**< Integer vector to be written into file. */
							int n,							/**< Length of vector. */
							const char* datafilename,		/**< Data file name. */
							BooleanType append = BT_FALSE	/**< Indicates if integer shall be appended if the file already exists (otherwise it is overwritten). */
							);

/** Writes a real_t matrix/vector into a Matlab binary file.
 * \return SUCCESSFUL_RETURN \n
		   RET_INVALID_ARGUMENTS
 		   RET_UNABLE_TO_WRITE_FILE */
returnValue writeIntoMatFile(	FILE* const matFile,		/**< Pointer to Matlab binary file. */
								const real_t* const data,	/**< Data to be written into file. */
								int nRows,					/**< Row number of matrix. */
								int nCols, 					/**< Column number of matrix. */
								const char* name			/**< Matlab name of matrix/vector to be stored. */
								);

/** Writes in integer matrix/vector into a Matlab binary file.
 * \return SUCCESSFUL_RETURN \n
		   RET_INVALID_ARGUMENTS
 		   RET_UNABLE_TO_WRITE_FILE */
returnValue writeIntoMatFile(	FILE* const matFile,		/**< Pointer to Matlab binary file. */
								const int* const data,		/**< Data to be written into file. */
								int nRows,					/**< Row number of matrix. */
								int nCols,					/**< Column number of matrix. */
								const char* name			/**< Matlab name of matrix/vector to be stored. */
								);


/** Returns the current system time.
 * \return current system time */
real_t getCPUtime( );


/** Returns the N-norm of a vector.
 * \return 0: successful */
real_t getNorm(	const real_t* const v,	/**< Vector. */
				int n,					/**< Vector's dimension. */
				int type = 2			/**< Norm type, 1: one-norm, 2: Euclidean norm. */
				);


/** Tests whether two real_t-valued arguments are (numerically) equal.
 * \return	BT_TRUE:  arguments differ not more than TOL \n
		 	BT_FALSE: arguments differ more than TOL */
inline BooleanType isEqual(	real_t x,			/**< First real number. */
							real_t y,			/**< Second real number. */
							real_t TOL = ZERO	/**< Tolerance for comparison. */
							);


/** Tests whether a real_t-valued argument is (numerically) zero.
 * \return	BT_TRUE:  argument differs from 0.0 not more than TOL \n
		 	BT_FALSE: argument differs from 0.0 more than TOL */
inline BooleanType isZero(	real_t x,			/**< Real number. */
							real_t TOL = ZERO	/**< Tolerance for comparison. */
							);


/** Returns sign of a real_t-valued argument.
 * \return	 1.0: argument is non-negative \n
		 	-1.0: argument is negative */
inline real_t getSign(	real_t arg	/** real_t valued argument whose sign is to be determined. */
						);


/** Returns maximum of two integers.
 * \return	Maximum of two integers */
inline int getMax(	int x,	/**< First integer. */
					int y	/**< Second integer. */
					);
					
/** Returns minimum of two integers.
 * \return	Minimum of two integers */
inline int getMin(	int x,	/**< First integer. */
					int y	/**< Second integer. */
					);

					
/** Returns maximum of two reals.
 * \return	Maximum of two reals */
inline real_t getMax(	real_t x,	/**< First real number. */
						real_t y	/**< Second real number. */
						);

/** Returns minimum of two reals.
 * \return	Minimum of two reals */
inline real_t getMin(	real_t x,	/**< First real number. */
						real_t y	/**< Second real number. */
						);

/** Returns the absolute value of a real number.
 * \return	Absolute value of a real number */
inline real_t getAbs(	real_t x	/**< Real number. */
						);

/** Returns the square-root of a real number.
 * \return	Square-root of a real number */
inline real_t getSqrt(	real_t x	/**< Non-negative real number. */
						);


/** Computes "residual" of KKT system.  */
void getKKTResidual(	int nV,						/**< Number of variables. */
						int nC,						/**< Number of constraints. */
						const real_t* const H,		/**< Hessian matrix. */
						const real_t* const g,		/**< Sequence of gradient vectors. */
						const real_t* const A,		/**< Constraint matrix. */
						const real_t* const lb,		/**< Sequence of lower bound vectors (on variables). */
						const real_t* const ub,		/**< Sequence of upper bound vectors (on variables). */
						const real_t* const lbA,	/**< Sequence of lower constraints' bound vectors. */
						const real_t* const ubA,	/**< Sequence of upper constraints' bound vectors. */
						const real_t* const x,		/**< Sequence of primal trial vectors. */
						const real_t* const y,		/**< Sequence of dual trial vectors. */
						real_t& stat,				/**< Maximum value of stationarity condition residual. */
						real_t& feas,				/**< Maximum value of primal feasibility violation. */
						real_t& cmpl				/**< Maximum value of complementarity residual. */
						);

/** Computes "residual" of KKT system (for simply bounded QPs).  */
void getKKTResidual(	int nV,						/**< Number of variables. */
						const real_t* const H,		/**< Hessian matrix. */
						const real_t* const g,		/**< Sequence of gradient vectors. */
						const real_t* const lb,		/**< Sequence of lower bound vectors (on variables). */
						const real_t* const ub,		/**< Sequence of upper bound vectors (on variables). */
						const real_t* const x,		/**< Sequence of primal trial vectors. */
						const real_t* const y,		/**< Sequence of dual trial vectors. */
						real_t& stat,				/**< Maximum value of stationarity condition residual. */
						real_t& feas,				/**< Maximum value of primal feasibility violation. */
						real_t& cmpl				/**< Maximum value of complementarity residual. */
						);


/** Writes a value of BooleanType into a string.
 * \return SUCCESSFUL_RETURN */
returnValue convertBooleanTypeToString(	BooleanType value, 		/**< Value to be written. */
										char* const string		/**< Input: String of sufficient size, \n
																	 Output: String containing value. */
										);

/** Writes a value of SubjectToStatus into a string.
 * \return SUCCESSFUL_RETURN */
returnValue convertSubjectToStatusToString(	SubjectToStatus value,	/**< Value to be written. */
											char* const string		/**< Input: String of sufficient size, \n
																		 Output: String containing value. */
											);

/** Writes a value of PrintLevel into a string.
 * \return SUCCESSFUL_RETURN */
returnValue convertPrintLevelToString(	PrintLevel value, 		/**< Value to be written. */
										char* const string		/**< Input: String of sufficient size, \n
																	 Output: String containing value. */
										);


/** Converts a returnValue from an (S)QProblem(B) object into a more 
 *	simple status flag.
 *
 * \return  0: QP problem solved
 *          1: QP could not be solved within given number of iterations
 *         -1: QP could not be solved due to an internal error
 *         -2: QP is infeasible (and thus could not be solved)
 *         -3: QP is unbounded (and thus could not be solved)
 */
int getSimpleStatus(	returnValue returnvalue, 				/**< ReturnValue to be analysed. */
						BooleanType doPrintStatus = BT_FALSE	/**< Flag indicating whether simple status shall be printed to screen. */
						);




#ifdef __DEBUG__
/** Writes matrix with given dimension into specified file. */
extern "C" void gdb_printmat(	const char *fname,			/**< File name. */
								real_t *M,					/**< Matrix to be written. */
								int n,						/**< Number of rows. */
								int m,						/**< Number of columns. */
								int ldim					/**< Leading dimension. */
								);
#endif /* __DEBUG__ */


END_NAMESPACE_QPOASES


#include <optec/Utils.ipp>

#endif	/* QPOASES_UTILS_HPP */


/*
 *	end of file
 */
