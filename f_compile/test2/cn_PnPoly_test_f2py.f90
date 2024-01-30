SUBROUTINE Calculate_idx_in_frame(n_pts, n_ply, pts, plys, idx)
! f2py intent(in)  :: pts, plys, n_ply, n_pts
! f2py intent(out) :: idx
    IMPLICIT NONE 
    INTEGER :: n_pts, n_ply, j, ii, idx(n_pts), count
    REAL :: pts(n_pts,2), plys(n_ply,2), count_pct
    ! REAL, ALLOCATABLE :: plys(:,:)
    ! f2py -m calc -c ./cn_PnPoly_test_f2py.f90

    ii = 0
    PCD_pts:DO j=1,n_pts
        count = count+1
        CALL Cn_PnPoly_CC(pts(j,1:2), plys, n_ply, ii)
        idx(j) = ii

    END DO PCD_pts

END SUBROUTINE Calculate_idx_in_frame

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