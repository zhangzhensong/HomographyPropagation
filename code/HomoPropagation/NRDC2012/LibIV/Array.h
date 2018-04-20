/*
	============================
	Array       2009/11/11
	============================

	Array classes
	Array1D         one dimension
	Array2D         two dimensions

	============================
	Modified on 2009/12/08/
	Add FastArray3D
	
*/
#pragma once;

#include <LibIV/libivexception.h>
#include <LibIV/Tuple.h>
#include <LibIV/CppHelpers.h>
#include <LibIV/Types.h>

using namespace LibIV::Math;

#include <vector>

namespace LibIV
{
	namespace Memory
	{
		namespace Array
		{
			// =================
			// Check
			class CheckNop
			{
			public:
				static void checkEmpty(const void * )				{}
				static void checkPointer(const void *)				{}
				static void checkAllocation(const void *)			{}
				static void checkAccess(const void *,int,uint)		{}
			};// End of CheckNop

			class CheckAll
			{
			public:
				static void checkEmpty(const void *p)				{if(p!=NULL) LIBIV_EXCEPTION_WITH_ERROR(Exception,"Memory - already initialized"); }
				static void checkPointer(const void *p)				{if(p==NULL) LIBIV_EXCEPTION_WITH_ERROR(Exception,"Memory - access violation");}
				static void checkAllocation(const void *p)			{if(p==NULL) LIBIV_EXCEPTION_WITH_ERROR(Exception,"Memory - allocation failed");}
				static void checkAccess(const void *p,int i,uint n) {if(p == NULL || i<0 || i>=(int)n) LIBIV_EXCEPTION_WITH_ERROR(Exception,"Memory - access out of bounds");}
			};// End of CheckAll


			// =======================================================
			// Array1D class
			template<typename T_Type, 
			class P_Check = CheckAll
			>
			class Array1D
			{
			private:
				T_Type * m_Data;
				uint     m_Size;
			public:
				// ==================
				// Constructors and de-constructor
				Array1D(void)
				{
					m_Data = NULL;
					m_Size = 0;
				}
				Array1D(uint size)
				{
					m_Data = NULL;
					m_Size = 0;
					allocate(size);
				}
				Array1D(const std::vector<T_Type>& vec)
				{
					m_Data = NULL;
					m_Size = 0;
					allocate(vec.size());
					ForIndex(i,vec.size())
					{
						P_Check::checkAccess(m_Data,i,m_Size);
						m_Data[i] = vec[i];
					}
				}
				Array1D(const Array1D& arr)
				{
					m_Data = NULL;
					m_Size = 0;
					if(arr.size() > 0)
					{
						allocate(arr.size());
						ForIndex(i,m_Size)
						{
							P_Check::checkAccess(m_Data,i,m_Size);
							m_Data[i] = arr[i];
						}
					}
				}

				~Array1D()
				{
					if(m_Data != NULL)
					{
						delete [] m_Data;
					}
				}

				Array1D& operator= (const Array1D& arr)
				{
					erase();
					if(arr.size() > 0)
					{
						allocate(arr.size());
						ForIndex(i,m_Size)
						{
							P_Check::checkAccess(m_Data,i,m_Size);
							m_Data[i] = arr[i];
						}
					}
					return (*this);
				}

				Array1D<T_Type>& operator*=(const Array1D<T_Type> & rhs)
				{   
					if(m_Size == rhs.size())
						for(uint i = 0;i < m_Size;i++)    
							m_Data[i] *= rhs.m_Data[i];
					return *this ;     
				}

				Array1D<T_Type> operator*(T_Type rhs)
				{
					Array1D<T_Type> tmp;
					tmp.set(m_Size);
					for(uint i = 0;i<m_Size;i++)
						tmp[i] = m_Data[i] * rhs;
					return tmp;
				}
				
				Array1D<T_Type> operator+(const Array1D<T_Type> & rhs)
				{
					Array1D<T_Type> tmp;
					tmp.set(m_Size);
					for(uint i = 0;i<m_Size;i++)
						tmp[i] = m_Data[i] + rhs[i];
					return tmp;
				}

				void fill(const T_Type& value_to_fill_with)
				{
					ForIndex(i,m_Size)
					{
						P_Check::checkAccess(m_Data,i,m_Size);
						m_Data[i] = value_to_fill_with;
					}
				}

				// ==================
				// Allocate memory for the array
				void allocate(uint size_to_allocate)
				{
					P_Check::checkEmpty(m_Data);
					m_Size = size_to_allocate;
					m_Data = new T_Type[m_Size];
					P_Check::checkAllocation(m_Data);
				}
				// Delete the array
				void erase()
				{
					if(m_Data != NULL)
					{
						delete [] m_Data;
						m_Data = NULL;
						m_Size = 0;
					}
				}


				void set(int n) {   
					if(m_Data && m_Size == n)   
						return;   

					if(m_Data)   
						erase();

					allocate(n);
				}

				// ==================
				// Get informations	
				uint			size()			const	{return m_Size;}
				
				bool			empty()			const	{return (m_Size == 0);}
				
				const T_Type *	raw()			const	{return m_Data;}
				
				T_Type *		raw()					{return m_Data;}

				// ==================
				// Access
				const T_Type& operator[](int i) const
				{
					//P_Check::checkAccess(m_Data,i,m_Size);
					//iv_dbg_assert(m_Data!=NULL && i>=0 && i < (int)m_Size);
					return (m_Data[i]);
				}
				
				T_Type& operator[](int i)
				{
					//P_Check::checkAccess(m_Data,i,m_Size);
					//iv_dbg_assert(m_Data!=NULL && i>=0 && i < (int)m_Size);
					return (m_Data[i]);
				}

				T_Type& operator()(int i)
				{ 
					return (m_Data[i]);	 
				} 

				const T_Type& operator()(int i) const 
				{ 
					return (m_Data[i]);	 
				} 

			};// End of Array1D	

			// =======================================================
			// Array2D class
			template<typename T_Type,
				class P_Check = CheckAll
			>
			class Array2D
			{
			private:
				Array1D<T_Type,P_Check> *		m_Array;
				uint							m_Rows;
				uint							m_Cols;

			public:
				// ==================
				// Constructors and de-constructor
				Array2D()
				{
					m_Array = NULL;
					m_Rows = 0;
					m_Cols = 0;
				}

				Array2D(uint rows,uint cols)
				{
					m_Array = NULL;
					m_Rows = 0;
					m_Cols = 0;
					allocate(rows,cols);
				}

				Array2D(const Array2D& arr)
				{
					m_Array = NULL;
					m_Rows = 0;
					m_Cols = 0;
					if(arr.rows() > 0 && arr.cols() > 0)
					{
						allocate(arr.rows(),arr.cols());
						ForIndex(i,m_Rows)
						{
							P_Check::checkAccess(m_Array,i,m_Rows);
							m_Array[i] =  arr[i];
						}
					}
				}

				~Array2D()
				{
					if(m_Array)
					{
						ForIndex(i,m_Rows)
							m_Array[i].erase();
						delete [] m_Array;
					}
				}

				Array2D& operator=(const Array2D& arr)
				{
					erase();
					if(arr.rows() > 0 && arr.cols() > 0)
					{
						allocate(arr.rows(),arr.cols());
						ForIndex(i,m_Rows)
						{
							P_Check::checkAccess(m_Array,i,m_Rows);
							m_Array[i] = arr[i];
						}
					}
					return (*this);
				}

				void fill(const T_Type& value_to_fill_with)
				{
					ForIndex(i,m_Rows)
					{
						P_Check::checkAccess(m_Array,i,m_Rows);
						m_Array[i].fill(value_to_fill_with);
					}
				}

				// ==================
				// Allocate
				void allocate(uint rows,uint cols)
				{
					P_Check::checkEmpty(m_Array);
					m_Rows = rows;
					m_Cols = cols;
					m_Array = new Array1D<T_Type,P_Check>[m_Rows];
					P_Check::checkAllocation(m_Array);
					ForIndex(i,m_Rows)
						m_Array[i].allocate(m_Cols);
				}
				// Delete
				void erase()
				{
					if(m_Array)
					{
						ForIndex(i,m_Rows)
							m_Array[i].erase();
						delete [] m_Array;
						m_Array = NULL;
						m_Rows = 0;
						m_Cols = 0;
					}
				}

				// ==================
				// Get informations
				uint rows()							const	{ return m_Rows; }
				uint cols()							const	{ return m_Cols; }
				
				bool empty()							const   { return (m_Rows == 0 && m_Cols == 0); }

				const Array1D<T_Type,P_Check>* raw()	const	{ return (m_Array); }
				Array1D<T_Type,P_Check>* raw()					{ return (m_Array); }
				

				
				// ==================
				// [] operator
				const Array1D<T_Type,P_Check>& operator[](int i) const
				{
					P_Check::checkAccess(m_Array,i,m_Rows);
					//iv_dbg_assert(m_Array!=NULL && i>=0 && i < (int)m_Rows);
					return (m_Array[i]);
				}

				Array1D<T_Type,P_Check>& operator[](int i)
				{
					P_Check::checkAccess(m_Array,i,m_Rows);
					//iv_dbg_assert(m_Array!=NULL && i>=0 && i < (int)m_Rows);
					return (m_Array[i]);
				}

				const T_Type& at(int l,int c) const
				{	
					//P_Check::checkAccess(m_Array,l,m_Rows);
					return (m_Array[l][c]);
				}
				T_Type& at(int l,int c)
				{
					//P_Check::checkAccess(m_Array,l,m_Rows);
					return (m_Array[l][c]);
				}

			};// End of Array2D

			// =======================================================
			// FastArray2D class
			template <typename T_Type>
			class FastArray2D
			{
			private:
				T_Type	**	m_Data;
				uint		m_Rows;
				uint		m_Cols;
			
			public:
				// ==================
				// Constructors and de-constructor
				FastArray2D()
				{
					m_Data = NULL;
					m_Rows = 0;
					m_Cols = 0;
				}
				FastArray2D(uint rows,uint cols)
				{
					m_Data = NULL;
					m_Rows = 0;
					m_Cols = 0;
					allocate(rows,cols);
				}

				FastArray2D(const FastArray2D& fastArr)
				{
					m_Data = NULL;
					m_Rows = 0;
					m_Cols = 0;
					if( fastArr.rows() > 0 && fastArr.cols() > 0 )
					{
						m_Rows = fastArr.rows();
						m_Cols = fastArr.cols();
						allocate(m_Rows,m_Cols);
						ForIndex(i,m_Rows)
							ForIndex(j,m_Cols)
								m_Data[i][j] = fastArr(j,i);
					}
				}

				~FastArray2D()
				{
					if(m_Data)
					{
						delete [] (m_Data[0]);
						delete [] m_Data;
					}
				}

				FastArray2D& operator=(const FastArray2D& fastArr)
				{
					erase();
					if( fastArr.rows() > 0 && fastArr.cols() > 0 )
					{
						m_Rows = fastArr.rows();
						m_Cols = fastArr.cols();
						allocate(m_Rows,m_Cols);
						ForIndex(i,m_Rows)
							ForIndex(j,m_Cols)
								m_Data[i][j] = fastArr(j,i);
					}
					return (*this);
				}
	
				FastArray2D& operator/=(const T_Type& value_to_div)
				{
					iv_dbg_assert(m_Data!=NULL);
					ForIndex(i,m_Rows)
						ForIndex(j,m_Cols)
							m_Data[i][j] /= value_to_div;

					return (*this);
				}

				FastArray2D& operator+=(const T_Type& value_to_add)
				{
					iv_dbg_assert(m_Data!=NULL);
					ForIndex(i,m_Rows)
						ForIndex(j,m_Cols)
							m_Data[i][j] += value_to_add;

					return (*this);
				}

				void fill(const T_Type& value_to_fill_with)
				{
					iv_dbg_assert(m_Data != NULL);
					ForIndex(i,m_Rows)
						ForIndex(j,m_Cols)
							m_Data[i][j] = value_to_fill_with;
				}

				// ==================
				// Allocate
				void allocate(uint rows,uint cols)
				{
					// Assert 1 !
					iv_dbg_assert(m_Data == NULL);
					m_Rows = rows;
					m_Cols = cols;
					m_Data = new T_Type*[rows];
					// Assert 2 !
					iv_dbg_assert(m_Data != NULL);
					m_Data[0] = new T_Type[rows * cols];
					// Assert 3 !
					iv_dbg_assert(m_Data[0] != NULL);
					ForRange(i,1,rows-1)
						m_Data[i] = m_Data[i-1] + cols;
				}
				
				// Delete the array
				void erase()
				{
					if(m_Data)
					{
						delete [] (m_Data[0]);
						delete [] m_Data;
						m_Data = NULL;
						m_Rows = 0;
						m_Cols = 0;
					}
				}

				void set(int lx, int ly) {   
					if(m_Data && m_Rows == ly && m_Cols == lx)   
						return;   

					if(m_Data)   
						erase();

					allocate(ly,lx);
				}

				// ==================
				// Get information
				uint rows()				const				{ return m_Rows; }
				uint cols()				const				{ return m_Cols; }
	
				bool empty()			const				{ return ( m_Rows == 0 && m_Cols == 0); }

				const T_Type** raw()	const				{ return m_Data; }
				T_Type **      raw()						{ return m_Data; }

				// ==================
				// Access

				const T_Type& at(uint r,uint c) const
				{
					//iv_dbg_assert((r<m_Rows && c<m_Cols && m_Data != NULL));
					return m_Data[r][c];
				}

				T_Type&       at(uint r,uint c) 
				{
					//iv_dbg_assert((r<m_Rows && c<m_Cols && m_Data != NULL));
					return m_Data[r][c];
				}

				const T_Type& operator()(int x, int y) const
				{ 
					return m_Data[y][x];	 
				}

				T_Type& operator()(int x, int y)
				{ 
					return m_Data[y][x];	 
				} 


				const T_Type& operator[](uint i) const
				{
					//iv_dbg_assert((r<m_Rows && m_Data!= NULL));
					return m_Data[i/m_Cols][i%m_Cols];
				}

				T_Type &	  operator[](uint i)
				{
					//iv_dbg_assert((r<m_Rows && m_Data!= NULL));
					return m_Data[i/m_Cols][i%m_Cols];
				}

				void clear(T_Type val = 0)
				{
					fill(val);
				}	


				// operators

				FastArray2D<T_Type>& operator*=(T_Type val)
				{   
					for(uint i = 0;i < m_Rows;i++)   
						for(uint j = 0;j<m_Cols;j++)
							m_Data[i][j] *= val;
					return *this ;     
				} 

				FastArray2D<T_Type>& operator+=(const FastArray2D<T_Type>& other) {   

					int m_len_x, m_len_y;   
					if (m_Cols <= other.m_Cols){ m_len_x=m_Cols;}else{m_len_x=other.m_Cols;}   
					if (m_Rows <= other.m_Rows){ m_len_y=m_Rows;}else{m_len_y=other.m_Rows;}   

					for(int y = 0 ; y < m_len_y ; y++)   
						for(int x = 0 ; x < m_len_x ;x++)   
							m_Data[y][x] += other.m_Data[y][x];

					return *this ;   
				}

			}; // End of FastArray2D

			// =======================================================
			// FastArray3D
			template <typename T_Type>
			class FastArray3D
			{
			private:
				T_Type	***		m_Data;
				uint			m_Rows;
				uint			m_Cols;
				uint            m_Pages;

			public:
				// ==================
				// Constructors and de-constructor
				FastArray3D()
				{
					m_Data = NULL;
					m_Rows = 0;
					m_Cols = 0;
					m_Pages = 0;
				}
				FastArray3D(uint pages,uint rows,uint cols)
				{
					m_Data = NULL;
					m_Rows = 0;
					m_Cols = 0;
					m_Pages = 0;
					allocate(pages,rows,cols);
				}

				FastArray3D(const FastArray3D& fastArr)
				{
					m_Data = NULL;
					m_Rows = 0;
					m_Cols = 0;
					m_Pages = 0;
					if( fastArr.rows() > 0 && fastArr.cols() > 0 && fastArr.pages() > 0)
					{
						m_Rows = fastArr.rows();
						m_Cols = fastArr.cols();
						m_Pages = fastArr.pages();
						allocate(m_Pages,m_Rows,m_Cols);
						ForIndex(i,m_Pages)
							ForIndex(j,m_Rows)
							ForIndex(k,m_Cols)
							m_Data[i][j][k] = fastArr[i][j][k];
					}
				}

				~FastArray3D()
				{
					if(m_Data)
					{
						delete [] (m_Data[0][0]);
						delete [] (m_Data[0]);
						delete [] (m_Data);
					}
				}

				FastArray3D& operator=(const FastArray3D& fastArr)
				{
					erase();
					if( fastArr.rows() > 0 && fastArr.cols() > 0 &&fastArr.pages() > 0)
					{
						m_Rows = fastArr.rows();
						m_Cols = fastArr.cols();
						m_Pages = fastArr.pages();
						allocate(m_Pages,m_Rows,m_Cols);
						ForIndex(i,m_Pages)
							ForIndex(j,m_Rows)
							ForIndex(k,m_Cols)
							m_Data[i][j][k] = fastArr[i][j][k];
					}
					return (*this);
				}

				void fill(const T_Type& value_to_fill_with)
				{
					iv_dbg_assert(m_Data != NULL);
					ForIndex(i,m_Pages)
						ForIndex(j,m_Rows)
						ForIndex(k,m_Cols)
						m_Data[i][j][k] = value_to_fill_with;
				}

				// ==================
				// Allocate
				void allocate(uint pages,uint rows,uint cols)
				{
					// Assert 1 !
					iv_dbg_assert(m_Data == NULL);
					m_Rows = rows;
					m_Cols = cols;
					m_Pages = pages;

					m_Data = new T_Type**[pages];
					m_Data[0] = new T_Type*[pages*rows];
					m_Data[0][0] = new T_Type[pages*rows*cols];
					// Assert 2 !
					iv_dbg_assert(m_Data != NULL);
					iv_dbg_assert(m_Data[0] != NULL);
					iv_dbg_assert(m_Data[0][0] != NULL);

					ForRange(i,1,pages-1)
					{
						m_Data[i] = m_Data[i-1] + rows;
						m_Data[i][0] = m_Data[i-1][0] + rows * cols;
					}

					ForIndex(i,pages)
						ForRange(j,1,rows-1)
						m_Data[i][j] = m_Data[i][j-1] + cols;
				}

				// Delete the array
				void erase()
				{
					if(m_Data)
					{
						delete [] (m_Data[0][0]);
						delete [] (m_Data[0]);
						delete [] (m_Data);
						m_Data = NULL;
						m_Rows = 0;
						m_Cols = 0;
						m_Pages = 0;
					}
				}

				void set(int lx,int ly, int lz)
				{
					if(m_Data && m_Rows == ly && m_Cols == lx && m_Pages == lz)
						return;
					if(m_Data)
						erase();
					allocate(lz,ly,lx);
				}

				// ==================
				// Get information
				uint rows ()			const				{ return m_Rows; }
				uint cols ()			const				{ return m_Cols; }
				uint pages()            const               { return m_Pages;}

				bool empty()			const				{ return ( m_Rows == 0 && m_Cols == 0 && m_Pages == 0); }

				const T_Type*** raw()	const				{ return m_Data; }
				T_Type ***      raw()						{ return m_Data; }

				// ==================
				// Access

				const T_Type& at(uint p,uint r,uint c) const
				{
					//iv_dbg_assert((p < m_Pages && r<m_Rows && c<m_Cols && m_Data != NULL));
					return m_Data[p][r][c];
				}

				T_Type&       at(uint p,uint r,uint c) 
				{
					//iv_dbg_assert((r<m_Rows && c<m_Cols && m_Data != NULL));
					return m_Data[p][r][c];
				}

				const T_Type& operator()(int x, int y, int z) const
				{ 
					return m_Data[z][y][x];	 
				}

				T_Type& operator()(int x, int y, int z)
				{ 
					return m_Data[z][y][x];	 
				} 

				T_Type** operator[](uint p) const
				{
					//iv_dbg_assert((p<m_Pages && m_Data!= NULL));
					return m_Data[p];
				}

				T_Type **	  operator[](uint p)
				{
					//iv_dbg_assert((p<m_Pages && m_Data!= NULL));
					return m_Data[p];
				}

				void clear(T_Type val)
				{
					fill(val);
				}
			}; // End of FastArray3D
				
			typedef LibIV::Memory::Array::Array1D<int>    V1iArr;
			typedef LibIV::Memory::Array::Array1D<v2i>    V2iArr;
			typedef LibIV::Memory::Array::Array1D<v3i>    V3iArr;
			typedef LibIV::Memory::Array::Array1D<v5i>    V5iArr;
			typedef LibIV::Memory::Array::Array1D<double> V1dArr;
			typedef LibIV::Memory::Array::Array1D<v3d>    V3dArr;
			typedef LibIV::Memory::Array::Array1D<v4d>    V4dArr;

			typedef LibIV::Memory::Array::FastArray2D<int>    TensorInt;
			typedef LibIV::Memory::Array::FastArray2D<v2i>    TensorV2i;
            typedef LibIV::Memory::Array::FastArray2D<v2d>    TensorV2d;
			typedef LibIV::Memory::Array::FastArray2D<double> TensorDb;
			typedef LibIV::Memory::Array::FastArray2D<v3i>    TensorV3i;
			typedef LibIV::Memory::Array::FastArray2D<v4i>    TensorV4i;
			typedef LibIV::Memory::Array::FastArray2D<v3d>    TensorV3D;
			typedef LibIV::Memory::Array::FastArray2D<char>   TensorByte;

			inline void NormalizeArrayDouble1D(Array1D<double> & arr)
			{ 
				double maxVal = -1e30; 
				for(uint i = 0;i<arr.size();i++) 
				{ 
					if(arr[i] > maxVal) 
						maxVal = arr[i]; 
				}
 
				for(uint i = 0;i<arr.size();i++) 
				{ 
					arr[i] /= maxVal; 
				} 
			}

		} // End of array
	} // End of memory
} // End of LibIV