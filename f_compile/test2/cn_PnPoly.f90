subroutine  cn_PnPoly(P, V, n, r) ! P - Point; V - Polygon
! f2py intent(in) ::P,V,n
! f2py intent(out) ::r
    implicit none
    integer n, cn, i, r
    real*8 P(2), V(n,2), vt
    ! n = len(V)
    cn = 0    ! the  crossing number counter

    ! loop through all edges of the polygon
    ! i = 0
    ! while i<n: ! edge from V[i]  to V[i+1]
    do i =1, n-1
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
    end do
    r = iand(cn, 1)
    !print str(cn)
    ! return (cn iand 1)    ! 0 if even (out), and 1 if  odd (in)
END subroutine cn_PnPoly

subroutine  cn_PnPoly_CC(P, V, n, r) ! P - Point; V - Polygon
! f2py intent(in) ::P,V,n
! f2py intent(out) ::r
    implicit none
    integer n, cn, i, r
    real*8 P(2), V(n,2), vt
    logical inside 
    inside = .false.
    ! n = len(V)
    cn = 0    ! the  crossing number counter

    ! loop through all edges of the polygon
    ! i = 0
    ! while i<n: ! edge from V[i]  to V[i+1]
    do i =1, n
      ! upward crossing or downward crossing
      if ( ( V(i,2) <= P(2) .and.  V(i+1,2) > P(2) ) .or. ( V(i,2)>P(2) .and. V(i+1,2)<=P(2) )  ) THEN
        vt = (P(1)  - V(i,1)) * (V(i+1,2) - V(i,2)) - (V(i+1,1) - V(i,1))*(P(2)  - V(i,2))
        if (V(i+1,2)<V(i,2)) THEN 
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
end subroutine cn_PnPoly_CC