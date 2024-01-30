PROGRAM MAIN
    IMPLICIT NONE 
    INTEGER :: i, j, n_ply, n_pcd, n_pts, r, totalN, count,count_mod=1D7
    REAL :: pts(4), count_pct
    REAL, ALLOCATABLE :: plys(:,:)
    CHARACTER(len=100) :: line_cur, line_last
    CHARACTER(len=100) :: boundary_name, line_input, line_output
    CHARACTER(len=100) :: inputfile, outputfile, filename

    write(*,*) '                                                             ' 
    write(*,*) '*************************************************************' 
    write(*,*) '*************************************************************' 
    write(*,*) '                                                             ' 
    write(*,*) '                Processing the boundary file:                '
    write(*,*) '                                                             ' 
    write(*,*) '*************************************************************' 
    write(*,*) '*************************************************************' 
    write(*,*) '                                                             ' 

    n_ply = 0
    filename = 'boundary.pt'
    CALL Count_lines(filename, n_ply)
    write(*,'(a)',advance='no') 'The number of ply points is :'
    write(*,*) n_ply
    write(*,*) ' '

    ALLOCATE(plys(n_ply,2))
    CALL LOAD_boundary(n_ply, plys)

    write(*,*) '                                                             ' 
    write(*,*) '*************************************************************' 
    write(*,*) '*************************************************************' 
    write(*,*) '                                                             ' 
    write(*,*) '                  Processing the PCD files:                  '
    write(*,*) '                                                             ' 
    write(*,*) '*************************************************************' 
    write(*,*) '*************************************************************' 
    write(*,*) '                                                             ' 

    filename = 'input.path'
    CALL Count_lines(filename, n_pcd)
    write(*,'(a)',advance='no') 'The number of pcd files is :'
    write(*,*) n_pcd
    write(*,*) ' '


    totalN = 0
    
    OPEN(UNIT=13, FILE='input.path',action='READ', STATUS='OLD')
    OPEN(UNIT=14, FILE='output.path',action='READ', STATUS='OLD')
    PCD_files: DO i =1, n_pcd
        write(*,*) '                                    ' 
        write(*,*) '====================================' 
        write(*,*) 'Start to process a new PCD file: ...'
        write(*,*) '====================================' 
        write(*,*) '                                    ' 

        read(13, '(a)') line_input
        read(14, '(a)') line_output

        inputfile = trim(line_input)
        outputfile = trim(line_output)

        write(*,'(a)',advance='no') 'input PCD file:  ' 
        write(*,*), inputfile
        write(*,*) ' '
        write(*,'(a)',advance='no') 'output file prefix: ' 
        write(*,*), outputfile
        write(*,*) ' '

        OPEN(UNIT=21, FILE=inputfile,action='READ', STATUS='OLD')
        OPEN(UNIT=22, FILE=outputfile,action='WRITE')
        
        PCD_header: DO j = 1, 11
            read(21, '(a)') line_cur
            ! write(*,*), line_cur
            if (j ==10) THEN
                write(*,'(a)',advance='no'), 'Total number of points in the pcd is: '
                write(*,*), TRIM(ADJUSTL(line_cur(8:28)))
                write(*,*) ' '
                line_cur = TRIM(ADJUSTL(line_cur(8:28)))
                read(line_cur,*), totalN
            end if
        END DO PCD_header

        write(*,*) '------------------------------------' 
        write(*,*) ' Begin to estimate each points:...   '
        write(*,*) '------------------------------------' 
        
        count = 0
        PCD_pts:DO j=1,totalN
            count = count+1
            read(21, *) pts(1), pts(2), pts(3), pts(4)
            ! write(*,*), pts(1), pts(2), pts(3), pts(4) 
            CALL Cn_PnPoly_CC(pts(1:2), plys, n_ply, r)

            IF (mod(count, count_mod)==0) THEN
                count_pct = 100.0*count/(1.0*totalN)
                write(*,100), count_pct
            END if

            write(22,101), r

        END DO PCD_pts
        
        write(*,*) '------------------------------------' 
        write(*,*) '       Finish the estimating !    '
        write(*,*) '------------------------------------' 
        CLOSE(UNIT=21)
        CLOSE(UNIT=22)

    END DO PCD_files

    DEALLOCATE(plys)
    CLOSE(UNIT=13)
    CLOSE(UNIT=14)

    100 FORMAT('        progress: ' , F5.1, '%') 
    101 FORMAT(I1) 
END PROGRAM MAIN

SUBROUTINE  Cn_PnPoly_CC(P, V, n_ply, r) ! P - Point; V - Polygon
    implicit none
    integer n_ply, cn, i, r
    real P(2), V(n_ply,2), vt
    logical inside 
    inside = .false.
    ! n = len(V)
    cn = 0    ! the  crossing number counter
    ! loop through all edges of the polygon
    ! i = 0
    ! while i<n: ! edge from V[i]  to V[i+1]
    do i =2, n_ply
        ! upward crossing or downward crossing
        if ( ( V(i,2) <= P(2) .and.  V(i-1,2) > P(2) ) .or. ( V(i,2)>P(2) .and. V(i-1,2)<=P(2) )  ) THEN
            vt = (P(1)  - V(i,1)) * (V(i-1,2) - V(i,2)) - (V(i-1,1) - V(i,1))*(P(2)  - V(i,2))
            if (V(i-1,2)<V(i,2)) THEN 
                vt = -vt
            end if 
            if (vt<0) THEN 
                inside = .not. inside
            end if
        end if
    end do
    if (inside) THEN
        r = 1
    else
        r = 0
    end if
END SUBROUTINE Cn_PnPoly_CC


SUBROUTINE  cn_PnPoly(P, V, n, r) ! P - Point; V - Polygon
    implicit none
    integer n, cn, i, r
    real P(2), V(n,2), vt
    ! n = len(V)
    cn = 0    ! the  crossing number counter

    ! loop through all edges of the polygon
    ! i = 0
    ! while i<n: ! edge from V[i]  to V[i+1]
    DO i =1, n-1
        ! upward crossing or downward crossing
        if ( ( V(i,2) <= P(2) .and.  V(i+1,2) > P(2) ) .or. ( V(i,2)>P(2) .and. V(i+1,2)<=P(2) )  ) THEN
        ! compute  the actual edge-ray intersect x-coordinate
            vt = (P(2)  - V(i,2)) / (V(i+1,2) - V(i,2))
        ! if P[0] > (V[i][0] + vt * (V[i+1][0] - V[i][0])): ! P.x > intersect - ray toward left
            if (P(1) < ( V(i,1) + vt*( V(i+1,1)-V(i,1) ) ) ) THEN ! P.x < intersect - ray toward right - original
                cn = cn+1   ! a valid crossing of y=P[1] right of P.x
            end if
        end if
        ! i = i+ 1
    END DO
    r = iand(cn, 1)
    !print str(cn)
    ! return (cn iand 1)    ! 0 if even (out), and 1 if  odd (in)
END SUBROUTINE cn_PnPoly


SUBROUTINE Count_lines(filename, n_ply)
    IMPLICIT NONE 
    INTEGER :: IOstatus, n_ply
    CHARACTER(len=100) :: line_cur, filename

    filename = trim(filename)
    OPEN(UNIT=11, FILE=filename,action='READ', STATUS='OLD')
    n_ply = 0
    lines: DO
        read(11, '(a)', IOSTAT=IOstatus) line_cur
        IF (IOstatus<0) THEN
            EXIT
        END IF
        n_ply = n_ply+1
    END DO lines
    CLOSE(UNIT=11)

END SUBROUTINE Count_lines


SUBROUTINE LOAD_boundary(n_ply, plys)
    IMPLICIT NONE 
    INTEGER :: n_ply, i 
    REAL :: plys(n_ply,2)

    write(*,*) 'The ply points are :'
    OPEN(UNIT=11, FILE='boundary.pt',action='READ', STATUS='OLD')
    lines: DO i =1,n_ply
        read(11, *) plys(i,1), plys(i,2)
        write(*, *), plys(i,1), plys(i,2)
    END DO lines
    CLOSE(UNIT=11)

END SUBROUTINE LOAD_boundary