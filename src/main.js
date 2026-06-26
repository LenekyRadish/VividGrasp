import "./styles.css";

const replayButton = document.querySelector("#replayButton");
const pybulletRender = document.querySelector("#pybulletRender");

replayButton.addEventListener("click", () => {
  const url = new URL(pybulletRender.src);
  url.searchParams.set("run", Date.now().toString());
  pybulletRender.src = url.toString();
});
