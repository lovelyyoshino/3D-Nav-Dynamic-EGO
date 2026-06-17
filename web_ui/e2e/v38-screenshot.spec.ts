import { expect, test } from "@playwright/test";

/**
 * Live screenshot proof for v3.8 multi-goal placeholder + bridge planned legs.
 * Requires `ros2 launch nav3d_ros2_bridge nav3d_bridge.launch.py` running on
 * port 9090 with the building2_9 PCD loaded, plus `npm run dev` on 5173.
 */
test("v3.8 multi-goal placeholder + bridge planned proof", async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== "desktop", "desktop-only");
  test.setTimeout(120_000);

  await page.goto("/");
  await expect(page.getByLabel("导航状态").getByText("已连接")).toBeVisible({ timeout: 30000 });
  // 62901 voxels render is heavy — wait long enough for the InstancedMesh to land.
  await expect(page.getByText("62901 体素")).toBeVisible({ timeout: 30000 });
  // Make sure the right-dock is mounted before clicking 起点 / 设置起点.
  await expect(page.getByRole("heading", { name: "起点 / 目标" })).toBeVisible({ timeout: 15000 });

  await page.getByRole("button", { name: "起点", exact: true }).click();
  await page.getByRole("spinbutton", { name: "X" }).fill("-13.00");
  await page.getByRole("spinbutton", { name: "Y" }).fill("8.00");
  await page.getByRole("spinbutton", { name: "Z" }).fill("1.00");
  await page.getByRole("button", { name: "设置起点", exact: true }).click();
  await expect(page.getByText("起点：已设")).toBeVisible({ timeout: 10000 });

  for (const goal of [
    { x: -5, y: 8, z: 1 },
    { x: 5, y: 8, z: 1 },
    { x: 13, y: 8, z: 1 },
  ]) {
    await page.getByRole("spinbutton", { name: "X" }).fill(goal.x.toFixed(2));
    await page.getByRole("spinbutton", { name: "Y" }).fill(goal.y.toFixed(2));
    await page.getByRole("spinbutton", { name: "Z" }).fill(goal.z.toFixed(2));
    await page.getByRole("button", { name: "添加目标" }).click();
  }
  await expect(page.getByText("3 目标")).toBeVisible();
  // No preview line authored on add — operator only sees goal markers.
  await expect(page.getByText("0 规划点")).toBeVisible();

  await page.waitForTimeout(800);
  await page.screenshot({ path: testInfo.outputPath("v38-pre-play-placeholders.png"), fullPage: false });
  const routePre = await page.evaluate(() => (window as any).__nav3dSceneProbe?.routePoints?.() ?? []);
  console.log("ROUTE_LEN_PRE=" + routePre.length);

  await page.getByRole("button", { name: "启动", exact: true }).click();
  await expect(page.locator(".bridge-status").getByText(/plan_success/)).toBeVisible({ timeout: 12000 });
  await page.waitForTimeout(1500);

  await page.screenshot({ path: testInfo.outputPath("v38-post-play-leg0-planned.png"), fullPage: false });
  const routePost = await page.evaluate(() => (window as any).__nav3dSceneProbe?.routePoints?.() ?? []);
  console.log("ROUTE_LEN_POST=" + routePost.length);
  expect(routePost.length).toBeGreaterThan(routePre.length);
});
